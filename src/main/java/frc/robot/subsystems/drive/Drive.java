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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.littletonUtils.AllianceFlipUtil;
import frc.lib.littletonUtils.EqualsUtil;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.PoseEstimator;
import frc.lib.littletonUtils.PoseEstimator.TimestampedVisionUpdate;
import frc.lib.littletonUtils.SwerveSetpointGenerator;
import frc.lib.littletonUtils.SwerveSetpointGenerator.SwerveSetpoint;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.util.AlertManager;
import frc.robot.util.BasicTrapezoid;
import frc.robot.util.MeshedDrivingController;
import frc.robot.util.SetpointGenerator;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.List;
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
  private Rotation2d lastGyroYaw = new Rotation2d();

  private Alert m_gyroDisconnectedAlert = new Alert("Gyro Disconnected", AlertType.kError);

  public enum DriveProfiles {
    kDefault,
    kPathplanner,
    kAutoAlign,
    kDriveToPoint,
    kIntakeMesh,
    kBargeMesh,
    kCharacterization,
    kMeshedUserControls,
    kStop
  }

  private SubsystemProfiles<DriveProfiles> m_profiles;

  private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds m_desiredAutoChassisSpeeds = null;

  private Rotation2d m_desiredHeading = new Rotation2d();

  private PIDController m_driveController =
      new PIDController(
          DriveConstants.kDriveToPointP.get(), DriveConstants.kDriveToPointI.get(), 0.0);

  private PIDController m_autoDriveController =
      new PIDController(
          DriveConstants.kDriveToPointAutoP.get(),
          DriveConstants.kDriveToPointAutoI.get(),
          DriveConstants.kDriveToPointAutoD.get());

  private PIDController m_autoIntakeDriveController =
      new PIDController(
          DriveConstants.kDriveToPointAutoIntakeP.get(),
          DriveConstants.kDriveToPointAutoIntakeI.get(),
          DriveConstants.kDriveToPointAutoIntakeD.get());

  private BasicTrapezoid m_driveTrapezoid =
      new BasicTrapezoid(
          MetersPerSecond.of(DriveConstants.kDriveToPointMaxVelocity.get()),
          MetersPerSecondPerSecond.of(DriveConstants.kDriveToPointMaxAcceleration.get()),
          MetersPerSecondPerSecond.of(DriveConstants.kDriveToPointMaxDeceleration.get()));

  private PIDController m_headingController =
      new PIDController(
          DriveConstants.kDriveToPointHeadingP.get(),
          DriveConstants.kDriveToPointHeadingI.get(),
          DriveConstants.kDriveToPointHeadingD.get());

  private LoggedTunableNumber m_ffMinRadiusTeleop = new LoggedTunableNumber("Min FF Radius", 100.0);
  private LoggedTunableNumber m_ffMaxRadiusTeleop = new LoggedTunableNumber("Max FF Radius", 100.0);
  private LoggedTunableNumber m_ffMinRadiusAuto =
      new LoggedTunableNumber("Min FF Radius Auto", 100.0);
  private LoggedTunableNumber m_ffMaxRadiusAuto =
      new LoggedTunableNumber("Max FF Radius Auto", 100.0);

  private double m_desiredAcceleration = 0.0;

  private Pose2d m_driveToPointTargetPose = null;

  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final PoseEstimator m_poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  // private final PoseEstimator m_poseEstimatorNoSlowVision =
  //     new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

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

  private MeshedDrivingController m_meshedController;

  private ChassisSpeeds m_userChassisSpeeds;

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
    periodicHash.put(DriveProfiles.kPathplanner, this::pathplannerPeriodic);
    periodicHash.put(DriveProfiles.kAutoAlign, this::autoAlignPeriodic);
    periodicHash.put(DriveProfiles.kDriveToPoint, this::driveToPointPeriodic);
    periodicHash.put(DriveProfiles.kIntakeMesh, this::intakeMeshPeriodic);
    periodicHash.put(DriveProfiles.kBargeMesh, this::bargeMeshPeriodic);
    periodicHash.put(DriveProfiles.kCharacterization, this::defaultPeriodic);
    periodicHash.put(DriveProfiles.kMeshedUserControls, this::meshedUserControlsPeriodic);
    periodicHash.put(DriveProfiles.kStop, this::stopPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, DriveProfiles.kDefault);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_swerveSetpointGenerator =
        new SwerveSetpointGenerator(
            DriveConstants.kDriveKinematics, DriveConstants.kModuleTranslations);

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController.setTolerance(Units.inchesToMeters(0.5));
    m_autoDriveController.setTolerance(Units.inchesToMeters(0.5));
    m_autoIntakeDriveController.setTolerance(Units.inchesToMeters(0.5));
    m_headingController.setTolerance(Units.degreesToRadians(1));

    AlertManager.registerAlert(m_gyroDisconnectedAlert);
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
          m_headingController.setP(DriveConstants.kDriveToPointHeadingP.get());
          m_headingController.setI(DriveConstants.kDriveToPointHeadingI.get());
          m_headingController.setD(DriveConstants.kDriveToPointHeadingD.get());
        },
        DriveConstants.kDriveToPointHeadingP,
        DriveConstants.kDriveToPointHeadingI,
        DriveConstants.kDriveToPointHeadingD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_driveController.setP(DriveConstants.kDriveToPointP.get());
          m_driveController.setI(DriveConstants.kDriveToPointI.get());
          m_driveController.setD(DriveConstants.kDriveToPointD.get());
        },
        DriveConstants.kDriveToPointP,
        DriveConstants.kDriveToPointI,
        DriveConstants.kDriveToPointD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_autoDriveController.setP(DriveConstants.kDriveToPointAutoP.get());
          m_autoDriveController.setI(DriveConstants.kDriveToPointAutoI.get());
          m_autoDriveController.setD(DriveConstants.kDriveToPointAutoD.get());
        },
        DriveConstants.kDriveToPointAutoP,
        DriveConstants.kDriveToPointAutoI,
        DriveConstants.kDriveToPointAutoD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_autoIntakeDriveController.setP(DriveConstants.kDriveToPointAutoIntakeP.get());
          m_autoIntakeDriveController.setI(DriveConstants.kDriveToPointAutoIntakeI.get());
          m_autoIntakeDriveController.setD(DriveConstants.kDriveToPointAutoIntakeD.get());
        },
        DriveConstants.kDriveToPointAutoIntakeP,
        DriveConstants.kDriveToPointAutoIntakeI,
        DriveConstants.kDriveToPointAutoIntakeD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_driveTrapezoid.setMaxVelocity(
              MetersPerSecond.of(DriveConstants.kDriveToPointMaxVelocity.get()));
          m_driveTrapezoid.setMaxAcceleration(
              MetersPerSecondPerSecond.of(DriveConstants.kDriveToPointMaxAcceleration.get()));
          m_driveTrapezoid.setMaxDeceleration(
              MetersPerSecondPerSecond.of(DriveConstants.kDriveToPointMaxDeceleration.get()));
        },
        DriveConstants.kDriveToPointMaxVelocity,
        DriveConstants.kDriveToPointMaxAcceleration,
        DriveConstants.kDriveToPointMaxDeceleration);

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
    Twist2d totalTwist = new Twist2d();
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
      Rotation2d deltaYaw;
      if (i == 0) {
        deltaYaw = m_rawGyroRotation.minus(lastGyroYaw);
      } else {
        deltaYaw = m_rawGyroRotation.minus(m_gyroInputs.odometryYawPositions[i - 1]);
      }
      var twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
      twist = new Twist2d(twist.dx, twist.dy, deltaYaw.getRadians());
      totalTwist =
          new Twist2d(
              totalTwist.dx + twist.dx, totalTwist.dy + twist.dy, totalTwist.dtheta + twist.dtheta);
    }

    if (m_gyroInputs.connected) {
      totalTwist.dtheta = m_gyroInputs.yawPosition.minus(lastGyroYaw).getRadians();
      lastGyroYaw = m_gyroInputs.yawPosition;
    } else {
      totalTwist.dtheta = m_rawGyroRotation.minus(lastGyroYaw).getRadians();
      lastGyroYaw = m_rawGyroRotation;
    }
    m_poseEstimator.addDriveData(Timer.getTimestamp(), totalTwist);
    // m_poseEstimatorNoSlowVision.addDriveData(Timer.getTimestamp(), totalTwist);

    // Logger.recordOutput("VisionSlow/Pose", m_poseEstimatorNoSlowVision.getLatestPose());
    // Logger.recordOutput("VisionSlow/Time", Timer.getTimestamp());

    // lets look for slip
    boolean slip = false;
    for (int i = 0; i < m_modules.length; i++) {
      double accel = m_modules[i].getDriveAcceleration();
      double current = m_modules[i].getDriveCurrent();
      if (current > 1) {
        Logger.recordOutput("ModuleOutputs/Module" + i + "/AmpsPerRotation", accel / current);
      } else {
        Logger.recordOutput("ModuleOutputs/Module" + i + "/AmpsPerRotation", 0.0);
      }
      if (Math.abs(accel * m_modules[i].getDriveVelocity()) > DriveConstants.kSlipThreshold.get()) {
        slip = true;
      }
      Logger.recordOutput("ModuleOutputs/Module" + i + "/curAccelRate", accel);
      Logger.recordOutput(
          "ModuleOutputs/Module" + i + "/curAccelRateTimesSpeed",
          Math.abs(accel * m_modules[i].getDriveVelocity()));
    }

    if (slip) {
      RobotState.getInstance().registerSlip();
    }

    Logger.recordOutput("Drive/Slip", slip);

    Logger.recordOutput(
        "Drive/FreeFall",
        m_gyroInputs.zAcceleration < DriveConstants.kFreefallAccelerationThreshold);

    Logger.recordOutput("Drive/Profile", m_profiles.getCurrentProfile());

    if (Constants.kUseAlerts && !m_gyroInputs.connected) {
      m_gyroDisconnectedAlert.set(true);
    } else {
      m_gyroDisconnectedAlert.set(false);
    }

    Logger.recordOutput("PeriodicTime/Drive", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void characterizationPeriodic() {
    Logger.recordOutput("Drive/DesiredHeading", m_desiredHeading.getDegrees());
    Logger.recordOutput("Drive/CurrentHeading", getPose().getRotation().getDegrees());
    Logger.recordOutput("Drive/DesiredSpeeds", new ChassisSpeeds());
  }

  public void defaultPeriodic() {
    if (m_profiles.getCurrentProfile() == DriveProfiles.kDriveToPoint) {
      runVelocity(m_desiredChassisSpeeds, MetersPerSecondPerSecond.of(m_desiredAcceleration));
    } else if (m_profiles.getCurrentProfile() != DriveProfiles.kCharacterization) {
      runVelocity(m_desiredChassisSpeeds);
    }

    Logger.recordOutput("Drive/DesiredHeading", m_desiredHeading.getDegrees());
    Logger.recordOutput("Drive/CurrentHeading", getPose().getRotation().getDegrees());
    Logger.recordOutput("Drive/DesiredSpeeds", m_desiredChassisSpeeds);
  }

  public void pathplannerPeriodic() {
    m_desiredChassisSpeeds = m_desiredAutoChassisSpeeds;
    if (m_desiredAutoChassisSpeeds == null) {
      m_desiredChassisSpeeds = new ChassisSpeeds();
      defaultPeriodic();
      return;
    }

    m_desiredChassisSpeeds = calculateAutoAlignSpeeds();

    defaultPeriodic();
  }

  public void meshedUserControlsPeriodic() {
    m_desiredChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_meshedController.calculateSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds(m_userChassisSpeeds, getRotation()),
                getPose()),
            getRotation());

    defaultPeriodic();
    if (RobotState.getInstance().getCurrentAction() == RobotAction.kCoralIntaking) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> {
            m_meshedController.setPIDControllers(
                DriveConstants.kDriveToIntakeMeshedP.get(),
                DriveConstants.kDriveToIntakeMeshedD.get(),
                DriveConstants.kDriveToIntakeThetaMeshedP.get(),
                DriveConstants.kDriveToIntakeThetaMeshedD.get());
          },
          DriveConstants.kDriveToIntakeMeshedP,
          DriveConstants.kDriveToIntakeMeshedD,
          DriveConstants.kDriveToIntakeThetaMeshedP,
          DriveConstants.kDriveToIntakeThetaMeshedD);
    }
  }

  public void autoAlignPeriodic() {
    m_desiredChassisSpeeds = calculateAutoAlignSpeeds();

    defaultPeriodic();
  }

  public void intakeMeshPeriodic() {
    // TODO: implement

    defaultPeriodic();
  }

  public void bargeMeshPeriodic() {
    // TODO: implement

    defaultPeriodic();
  }

  public void stopPeriodic() {
    runVelocity(new ChassisSpeeds());
  }

  private boolean m_hasNewTarget = false;

  public void driveToPointPeriodic() {
    Pose2d currentPose = getPose();

    double currentDistance =
        currentPose.getTranslation().getDistance(m_driveToPointTargetPose.getTranslation());

    if (EqualsUtil.epsilonEquals(0, m_driveToPointTargetPose.getX())) {
      Logger.recordOutput("Drive/ZeroTarget", Timer.getFPGATimestamp());
      m_desiredChassisSpeeds = new ChassisSpeeds();
      defaultPeriodic();

      return;
    }
    ChassisSpeeds currSpeeds = getChassisSpeeds();
    double currVelocity = Math.hypot(currSpeeds.vxMetersPerSecond, currSpeeds.vyMetersPerSecond);

    double ffMinRadius =
        DriverStation.isAutonomous() ? m_ffMinRadiusAuto.get() : m_ffMinRadiusTeleop.get();
    double ffMaxRadius =
        DriverStation.isAutonomous() ? m_ffMaxRadiusAuto.get() : m_ffMaxRadiusTeleop.get();

    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

    var state =
        m_driveTrapezoid.calculate(Meters.of(currentDistance), MetersPerSecond.of(currVelocity));
    double trapVelocity = state.velocity();
    double trapAccel = state.acceleration();

    m_desiredAcceleration = trapAccel * ffScaler;

    PIDController driveController =
        DriverStation.isAutonomous()
            ? (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoCoralIntaking
                ? m_autoIntakeDriveController
                : m_autoDriveController)
            : m_driveController;

    double driveVelocityScalar = -1 * trapVelocity * ffScaler;
    double pidVelocity = driveController.calculate(currentDistance, 0.0) * (1 - ffScaler);
    driveVelocityScalar += pidVelocity;
    if (currentDistance < driveController.getErrorTolerance()) {
      driveVelocityScalar = 0.0;
    }

    double headingError =
        currentPose.getRotation().minus(m_driveToPointTargetPose.getRotation()).getRadians();
    double headingVelocity =
        m_headingController.calculate(
            currentPose.getRotation().getRadians(),
            m_driveToPointTargetPose.getRotation().getRadians());

    if (Math.abs(headingError) < m_headingController.getErrorTolerance()) {
      headingVelocity = 0.0;
    }

    driveVelocityScalar =
        MathUtil.clamp(
            driveVelocityScalar,
            RobotState.getInstance().getCurrentAction() == RobotAction.kAutoCoralIntaking
                ? -DriveConstants.kMaxAutoIntakeSpeed.get()
                : -DriveConstants.kMaxAutoscoreSpeed.get(),
            RobotState.getInstance().getCurrentAction() == RobotAction.kAutoCoralIntaking
                ? DriveConstants.kMaxAutoIntakeSpeed.get()
                : DriveConstants.kMaxAutoscoreSpeed.get());

    // evil math
    // blame 254 for making this because i dont fully understand it
    // update: i understand it now :)
    Translation2d driveVelocity =
        new Pose2d(
                0.0,
                0.0,
                currentPose
                    .getTranslation()
                    .minus(m_driveToPointTargetPose.getTranslation())
                    .getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), headingVelocity, currentPose.getRotation());
    m_desiredChassisSpeeds = speeds;

    Logger.recordOutput("DriveToPoint/TargetPose", m_driveToPointTargetPose);
    Logger.recordOutput("DriveToPoint/DriveDistance", currentDistance);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);
    Logger.recordOutput(
        "DriveToPoint/TargetTheta",
        currentPose.getTranslation().minus(m_driveToPointTargetPose.getTranslation()).getAngle());

    Logger.recordOutput("DriveToPoint/PIDVelocity", pidVelocity);
    Logger.recordOutput("DriveToPoint/TrapezoidVelocity", trapVelocity);
    Logger.recordOutput("DriveToPoint/TrapezoidAccel", trapAccel);
    Logger.recordOutput("DriveToPoint/DriveVelocity", currVelocity);
    Logger.recordOutput("DriveToPoint/FFScaler", ffScaler);
    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);

    if (!DriverStation.isAutonomous()) {
      if (driveToPointWithinTolerance()) {
        updateProfile(getDefaultProfile());
      }
    } else {
      // TODO: hardcoded, debug later
      if (driveToPointWithinTolerance()
          && m_hasNewTarget
          && (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoAutoScore
              || RobotState.getInstance().getCurrentAction() == RobotAction.kAutoCoralOuttaking)
          && getPose()
                  .getTranslation()
                  .getDistance(AllianceFlipUtil.apply(FieldConstants.Reef.kCenter))
              < Units.inchesToMeters(72)) {
        m_hasNewTarget = false;
        Commands.runOnce(
                () -> {
                  updateProfile(DriveProfiles.kStop);
                })
            .andThen(
                Commands.waitUntil(
                    () -> {
                      RobotAction s = RobotState.getInstance().getCurrentAction();
                      return s != RobotAction.kAutoCoralOuttaking
                          && s != RobotAction.kAutoAutoScore;
                    }))
            .andThen(
                Commands.runOnce(
                    () -> {
                      updateProfile(DriveProfiles.kDriveToPoint);
                    }))
            .schedule();
        ;
      }
    }

    defaultPeriodic();
  }

  public ChassisSpeeds calculateAutoAlignSpeeds() {
    if (m_desiredHeading != null) {
      Logger.recordOutput("AutoAlignHeading", m_desiredHeading);
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
   * @param accel Acceleration in meters/sec^2
   */
  @SuppressWarnings("unused")
  public void runVelocity(ChassisSpeeds speeds, LinearAcceleration accel) {
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
      m_modules[i].runSetpoint(setpointStates[i], accel);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStatesOptimized);
    Logger.recordOutput("Drive/ChassisSpeedsSetpoint", m_currSetpoint.chassisSpeeds());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, MetersPerSecondPerSecond.of(0.0));
  }

  public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
    Logger.recordOutput("Drive/speed", speeds);
    m_desiredChassisSpeeds = speeds;
    m_userChassisSpeeds = speeds;
  }

  public void setDesiredAutoChassisSpeeds(ChassisSpeeds speeds) {
    m_desiredAutoChassisSpeeds = speeds;
  }

  public ChassisSpeeds getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setDesiredHeading(Rotation2d heading) {
    m_desiredHeading = heading;

    // driveToPointSetup();
  }

  public void setTargetPose(Pose2d pose) {
    m_hasNewTarget = true;
    m_driveToPointTargetPose = pose;
  }

  public Pose2d getTargetPose() {
    return m_driveToPointTargetPose;
  }

  /** Stops the drive. */
  public void stop() {
    updateProfile(DriveProfiles.kStop);
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
  @SuppressWarnings("unused")
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
    return m_poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
    // m_poseEstimatorNoSlowVision.resetPose(pose);
  }

  /**
   * Adds vision observations to the pose estimator.
   *
   * @param observations The vision observations to add.
   */
  public void addTimestampedVisionObservations(List<TimestampedVisionUpdate> observations) {
    // ChassisSpeeds speeds = getChassisSpeeds();
    // if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
    //     > DriveConstants.kVisionSpeedConstantK) {
    //   // Logger.recordOutput("VisionSlow/IgnoringVision", false);
    //   // m_poseEstimatorNoSlowVision.addVisionData(observations);
    // } else {
    //   // Logger.recordOutput("VisionSlow/IgnoringVision", true);
    // }
    m_poseEstimator.addVisionData(observations);
  }

  public void updateProfile(DriveProfiles newProfile) {
    m_profiles.setCurrentProfile(newProfile);
    if (newProfile == DriveProfiles.kDriveToPoint) {
      driveToPointPeriodic();
      // m_driveController.reset();
      // m_driveController.getErrorDerivative());
      // m_headingController.reset();
    }

    if (newProfile == DriveProfiles.kMeshedUserControls) {
      if (RobotState.getInstance().getCurrentAction() == RobotAction.kCoralIntaking) {
        Pair<Pose2d, List<Double>> desPose = SetpointGenerator.generateNearestIntake(getPose());
        m_meshedController =
            new MeshedDrivingController(
                desPose.getFirst(),
                true,
                DriveConstants.kDebounceAmount.get(),
                DriveConstants.kMeshDrivePriority.get());
        m_meshedController.setPIDControllers(
            DriveConstants.kDriveToIntakeMeshedP.get(),
            DriveConstants.kDriveToIntakeMeshedD.get(),
            DriveConstants.kDriveToIntakeThetaMeshedP.get(),
            DriveConstants.kDriveToIntakeThetaMeshedD.get());
        m_meshedController.setAxisLocked(
            desPose.getSecond().get(0),
            desPose.getSecond().get(1),
            desPose.getSecond().get(2),
            desPose.getSecond().get(3));
      } else {
        Pose2d desPose;
        if (RobotState.getInstance().getBargeLeftCage()) {
          desPose = SetpointGenerator.generateBargeLeft();
        } else {
          desPose = SetpointGenerator.generateBargeRight();
        }
        m_meshedController =
            new MeshedDrivingController(
                desPose,
                false,
                DriveConstants.kDebounceAmount.get(),
                DriveConstants.kMeshDrivePriority.get());
        m_meshedController.setPIDControllers(
            DriveConstants.kDriveToIntakeMeshedP.get(),
            DriveConstants.kDriveToIntakeMeshedD.get(),
            DriveConstants.kDriveToIntakeThetaMeshedP.get(),
            DriveConstants.kDriveToIntakeThetaMeshedD.get());
      }
    }
  }

  public DriveProfiles getDefaultProfile() {
    if (DriverStation.isAutonomous()) {
      return DriveProfiles.kDriveToPoint;
    } else {
      return DriveProfiles.kDefault;
    }
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
    if (m_profiles.getCurrentProfile() != DriveProfiles.kCharacterization) {
      updateProfile(DriveProfiles.kCharacterization);
    }
    for (int i = 0; i < 4; i++) {
      m_modules[i].runCharacterization(output);
    }
  }

  /** Returns the average velocity of the modules in radians/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += m_modules[i].getDriveVelocity() / 4.0;
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

  public boolean driveToPointWithinTolerance() {
    return driveToPointWithinTolerance(null, null);
  }

  public boolean driveToPointWithinTolerance(Distance linearTolerance, Angle headingTolerance) {
    if (linearTolerance == null) {
      linearTolerance =
          Meters.of(
              DriverStation.isAutonomous()
                  ? (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoCoralOuttaking
                      ? m_autoIntakeDriveController.getErrorTolerance()
                      : m_autoDriveController.getErrorTolerance())
                  : m_driveController.getErrorTolerance());
    }
    if (headingTolerance == null) {
      headingTolerance = Radians.of(m_headingController.getErrorTolerance());
    }
    return m_profiles.getCurrentProfile() != DriveProfiles.kDriveToPoint
        || (m_driveToPointTargetPose.getTranslation().getDistance(getPose().getTranslation())
                < linearTolerance.in(Meters)
            && m_driveToPointTargetPose.getRotation().minus(getRotation()).getMeasure().abs(Degrees)
                < headingTolerance.in(Degrees));
  }

  public double getYawVelocity() {
    return m_gyroInputs.yawVelocityRadPerSec;
  }
}
