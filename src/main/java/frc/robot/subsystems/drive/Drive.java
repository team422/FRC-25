// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.SwerveSetpointGenerator;
import frc.lib.littletonUtils.SwerveSetpointGenerator.SwerveSetpoint;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.DriveToPoint;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock m_odometryLock = new ReentrantLock();
  private final GyroIO m_gyroIO;
  public final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine m_sysId;

  public enum DriveProfiles {
    kDefault,
    kAutoAlign,
    kDriveToPoint,
  }

  private SubsystemProfiles<DriveProfiles> m_profiles;

  private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();

  private Rotation2d m_desiredHeading = new Rotation2d();
  private PIDController m_headingController =
      new PIDController(
          DriveConstants.kHeadingP.get(),
          DriveConstants.kHeadingI.get(),
          DriveConstants.kHeadingD.get());

  private DriveToPoint m_driveToPointCommand = new DriveToPoint(this, new Pose2d());
  private Pose2d m_driveToPointTargetPose = new Pose2d();

  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics, m_rawGyroRotation, m_lastModulePositions, new Pose2d());

  private SwerveSetpoint m_currSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private SwerveSetpointGenerator m_swerveSetpointGenerator;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, 0);
    m_modules[1] = new Module(frModuleIO, 1);
    m_modules[2] = new Module(blModuleIO, 2);
    m_modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    // Configure SysId
    m_sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    m_modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    Map<DriveProfiles, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(DriveProfiles.kDefault, this::defaultPeriodic);
    periodicHash.put(DriveProfiles.kAutoAlign, this::autoAlignPeriodic);
    periodicHash.put(DriveProfiles.kDriveToPoint, this::driveToPointPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, DriveProfiles.kDefault);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_swerveSetpointGenerator =
        new SwerveSetpointGenerator(
            DriveConstants.kDriveKinematics, DriveConstants.kModuleTranslations);
  }

  public void periodic() {
    double start = HALUtil.getFPGATime();

    m_odometryLock.lock(); // Prevents odometry updates while reading data
    m_gyroIO.updateInputs(m_gyroInputs);
    for (var module : m_modules) {
      module.updateInputs();
    }
    m_odometryLock.unlock();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_headingController.setP(DriveConstants.kHeadingP.get());
          m_headingController.setI(DriveConstants.kHeadingI.get());
          m_headingController.setD(DriveConstants.kHeadingD.get());
        },
        DriveConstants.kHeadingP,
        DriveConstants.kHeadingI,
        DriveConstants.kHeadingD);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : m_modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        m_modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = m_modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (m_gyroInputs.connected) {
        // Use the real gyro angle
        m_rawGyroRotation = m_gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
        m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      m_poseEstimator.updateWithTime(sampleTimestamps[i], m_rawGyroRotation, modulePositions);
    }

    Logger.recordOutput("Drive/Profile", m_profiles.getCurrentProfile());

    Logger.recordOutput("PeriodicTime/Drive", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void defaultPeriodic() {
    runVelocity(m_desiredChassisSpeeds);

    Logger.recordOutput("Drive/DesiredHeading", m_desiredHeading.getDegrees());
    Logger.recordOutput("Drive/CurrentHeading", getPose().getRotation().getDegrees());
    Logger.recordOutput("Drive/DesiredSpeeds", m_desiredChassisSpeeds);
  }

  public void autoAlignPeriodic() {
    m_desiredChassisSpeeds = calculateAutoAlignSpeeds();

    defaultPeriodic();
  }

  public void driveToPointPeriodic() {
    if (m_driveToPointCommand.isFinished()) {
      updateProfile(DriveProfiles.kDefault);
    }

    defaultPeriodic();
  }

  public ChassisSpeeds calculateAutoAlignSpeeds() {
    if (m_desiredHeading != null) {
      double output =
          m_headingController.calculate(
              getPose().getRotation().getRadians(), m_desiredHeading.getRadians());
      m_desiredChassisSpeeds.omegaRadiansPerSecond = output;
    }

    return m_desiredChassisSpeeds;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  @SuppressWarnings("unused")
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    // in real everything is backwards
    if (RobotBase.isReal() && DriveConstants.kRealReversed) {
      if (DriverStation.isTeleopEnabled()) {
        discreteSpeeds.vxMetersPerSecond = -discreteSpeeds.vxMetersPerSecond;
        discreteSpeeds.vyMetersPerSecond = -discreteSpeeds.vyMetersPerSecond;
      }
      discreteSpeeds.omegaRadiansPerSecond = -discreteSpeeds.omegaRadiansPerSecond;
    } else if (RobotBase.isSimulation() && DriveConstants.kSimReversed) {
      discreteSpeeds.vxMetersPerSecond = -discreteSpeeds.vxMetersPerSecond;
      discreteSpeeds.vyMetersPerSecond = -discreteSpeeds.vyMetersPerSecond;
      discreteSpeeds.omegaRadiansPerSecond = -discreteSpeeds.omegaRadiansPerSecond;
    }

    SwerveModuleState[] setpointStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

    m_currSetpoint =
        m_swerveSetpointGenerator.generateSetpoint(
            DriveConstants.kModuleLimitsFree, m_currSetpoint, discreteSpeeds, 0.02);
    SwerveModuleState[] setpointStatesOptimized = m_currSetpoint.moduleStates();

    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStatesOptimized);
    Logger.recordOutput("Drive/ChassisSpeedsSetpoint", m_currSetpoint.chassisSpeeds());
  }

  public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
    m_desiredChassisSpeeds = speeds;
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setDesiredHeading(Rotation2d heading) {
    m_desiredHeading = heading;
  }

  public void setTargetPose(Pose2d pose) {
    m_driveToPointTargetPose = pose;
    m_driveToPointCommand.setTargetPose(pose);
  }

  public Pose2d getTargetPose() {
    return m_driveToPointTargetPose;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void setCoast() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setBrakeMode(false);
    }
  }

  public void setBrake() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setBrakeMode(true);
    }
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.kModuleTranslations[i].getAngle();
    }
    DriveConstants.kDriveKinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param observation The VisionObservation object containing the vision data.
   */
  public void addVisionObservation(VisionObservation observation) {
    addVisionMeasurement(
        observation.visionPose(), observation.timestamp(), observation.standardDeviations());
  }

  public void updateProfile(DriveProfiles newProfile) {
    m_driveToPointCommand.cancel(); // cancel if it is already running

    if (newProfile == DriveProfiles.kDriveToPoint) {
      m_driveToPointCommand.schedule();
    }

    m_profiles.setCurrentProfile(newProfile);
  }

  public boolean headingWithinTolerance() {
    return Math.abs(m_headingController.getError()) < Units.degreesToRadians(5);
  }

  public void setModuleCurrentLimits(double supplyLimit) {
    for (var module : m_modules) {
      module.setCurrentLimits(supplyLimit);
    }
  }

  public DriveProfiles getCurrentProfile() {
    return m_profiles.getCurrentProfile();
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].runCharacterization(output);
    }
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += m_modules[i].getCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = m_modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public Rotation2d getGyroRotation() {
    return m_rawGyroRotation;
  }
}
