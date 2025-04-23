package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.littletonUtils.AllianceFlipUtil;
import frc.lib.littletonUtils.EqualsUtil;
import frc.lib.littletonUtils.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.Constants.AprilTagVisionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.Constants.FullTuningConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.Led.LedState;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.util.SetpointGenerator;
import frc.robot.util.SetpointGenerator.RobotSetpoint;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  // Subsystems
  private Drive m_drive;
  private Intake m_intake;
  private Indexer m_indexer;
  private Manipulator m_manipulator;
  private Climb m_climb;
  private Elevator m_elevator;
  private Led m_led;
  private AprilTagVision m_aprilTagVision;

  private AutoFactory m_autoFactory;

  public enum RobotAction {
    kTeleopDefault,
    kCoralIntaking,
    kCoralOuttaking,
    kAlgaeIntakingOuttaking,
    kAutoScore,
    kManualScore,
    kDriveToProcessor,
    kBargeScore,
    kBargeAutoScore,
    kBargeOuttaking,
    kProcessorOuttake,
    kCoralEject,
    kLollipopIntake,
    kCoralOTB,
    kCoralOTBL1,

    // algae descore sequence
    kAlgaeDescoringInitial,
    kAlgaeDescoringDeployManipulator,
    kAlgaeDescoringMoveUp,
    kAlgaeDescoringDriveAway,
    kAlgaeDescoringFinal,

    kAutoDefault,
    kAutoAutoScore,
    kAutoCoralIntaking,
    kAutoCoralOuttaking,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  private Timer m_timer = new Timer();
  private Timer m_secondaryTimer = new Timer();
  private Timer m_tertiaryTimer = new Timer();

  private ReefHeight m_desiredReefHeight = ReefHeight.L1;
  private int m_desiredBranchIndex = 0;
  private RobotSetpoint m_setpoint =
      SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight, true);

  private int m_desiredAlgaeIndex = 0;

  private boolean m_bargeLeftCage = true; // true - left cage; false - right cage

  private int m_numVisionGyroObservations = 0;

  private double m_odometryTrustFactor = 0.0;

  private boolean m_isBargeAuto = false;
  private boolean m_isCitrusAuto = false;

  private boolean m_autoTestingMode = false;

  // true - run through to funnel after intaking otb
  // false - hold position to score L1
  private boolean m_otbRunThrough = true;

  // only in ohio :skull:
  private boolean m_gamepieceUpdate = false;

  private Timer m_threadPriorityTimer = new Timer();

  private final List<RobotAction> kAlgaeDescoreSequence =
      List.of(
          RobotAction.kAlgaeDescoringInitial,
          RobotAction.kAlgaeDescoringDeployManipulator,
          RobotAction.kAlgaeDescoringMoveUp,
          RobotAction.kAlgaeDescoringFinal);

  private final List<RobotAction> kAlgaeDescoreSequenceAuto =
      List.of(
          RobotAction.kAlgaeDescoringInitial,
          RobotAction.kAlgaeDescoringDeployManipulator,
          RobotAction.kAlgaeDescoringMoveUp,
          RobotAction.kAlgaeDescoringDriveAway);

  // Singleton logic
  private static RobotState m_instance;

  private RobotState(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Manipulator manipulator,
      Climb climb,
      Elevator elevator,
      Led led,
      AprilTagVision aprilTagVision,
      AutoFactory autoFactory) {
    m_drive = drive;
    m_intake = intake;
    m_indexer = indexer;
    m_manipulator = manipulator;
    m_climb = climb;
    m_elevator = elevator;
    m_led = led;
    m_aprilTagVision = aprilTagVision;
    m_autoFactory = autoFactory;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    periodicHash.put(RobotAction.kAlgaeIntakingOuttaking, this::algaeIntakingOuttakingPeriodic);
    periodicHash.put(RobotAction.kCoralIntaking, this::coralIntakingPeriodic);
    periodicHash.put(RobotAction.kAutoScore, this::autoScorePeriodic);
    periodicHash.put(RobotAction.kManualScore, this::manualScorePeriodic);
    periodicHash.put(RobotAction.kDriveToProcessor, this::driveToProcessorPeriodic);
    periodicHash.put(RobotAction.kCoralOuttaking, this::coralOuttakingPeriodic);
    periodicHash.put(RobotAction.kProcessorOuttake, () -> {});
    periodicHash.put(RobotAction.kBargeScore, this::bargeScorePeriodic);
    periodicHash.put(RobotAction.kBargeAutoScore, this::bargeAutoScorePeriodic);
    periodicHash.put(RobotAction.kCoralEject, () -> {});
    periodicHash.put(RobotAction.kLollipopIntake, this::lollipopIntakePeriodic);
    periodicHash.put(RobotAction.kCoralOTB, this::coralIntakingPeriodic);
    periodicHash.put(RobotAction.kCoralOTBL1, this::coralOTBL1Periodic);
    periodicHash.put(RobotAction.kAlgaeDescoringInitial, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringDeployManipulator, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringMoveUp, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringDriveAway, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringFinal, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAutoAutoScore, this::autoAutoScorePeriodic);
    periodicHash.put(RobotAction.kAutoCoralIntaking, this::autoCoralIntakingPeriodic);
    periodicHash.put(RobotAction.kAutoCoralOuttaking, this::coralOuttakingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);

    m_threadPriorityTimer.start();
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public void triggerAlert() {
    m_led.updateState(LedState.kAlert);
  }

  public void cancelAlert() {
    m_led.cancelAlert();
  }

  public boolean getClimbAtSetpoint() {
    return m_climb.atSetpoint();
  }

  public boolean getWristAtScoringSetpoint() {

    return m_manipulator.atSetpoint();
  }

  public static RobotState startInstance(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Manipulator manipulator,
      Climb climb,
      Elevator elevator,
      Led led,
      AprilTagVision aprilTagVision,
      AutoFactory autoFactory) {
    if (m_instance == null) {
      m_instance =
          new RobotState(
              drive,
              intake,
              indexer,
              manipulator,
              climb,
              elevator,
              led,
              aprilTagVision,
              autoFactory);
    }
    return m_instance;
  }

  public void updateRobotState() {
    double start = HALUtil.getFPGATime();

    // Pose2d[] hardcoded =
    //     new Pose2d[] {
    //       new Pose2d(12.92, 1.96, Rotation2d.fromDegrees(90)),
    //       new Pose2d(12.92, 5.99, Rotation2d.fromDegrees(270)),
    //       new Pose2d(4.3519, 5.99, Rotation2d.fromDegrees(270)),
    //       new Pose2d(4.3519, 1.96, Rotation2d.fromDegrees(90)),
    //       new Pose2d(15.92385, 0.9075, Rotation2d.fromDegrees(126)),
    //       new Pose2d(
    //           15.92385, 7.1825, Rotation2d.fromDegrees(54).plus(Rotation2d.fromDegrees(180))),
    //       new Pose2d(1.6244, 7.1825, Rotation2d.fromDegrees(-54)),
    //       new Pose2d(
    //           1.6244, 0.9075, Rotation2d.fromDegrees(-126).plus(Rotation2d.fromDegrees(180))),
    //     };

    // Logger.recordOutput("Hardcoded", hardcoded);

    if (m_threadPriorityTimer != null && m_threadPriorityTimer.hasElapsed(20)) {
      Threads.setCurrentThreadPriority(true, 10);
      m_threadPriorityTimer = null;
    }

    m_profiles.getPeriodicFunctionTimed().run();

    if (Constants.kUseComponents) {
      updateComponent();
    }

    updateLED();

    Logger.recordOutput("RobotState/CurrentAction", m_profiles.getCurrentProfile());
    Logger.recordOutput("RobotState/TimerValue", m_timer.get());
    Logger.recordOutput("RobotState/SecondaryTimerValue", m_secondaryTimer.get());
    Logger.recordOutput("RobotState/TertiaryTimerValue", m_tertiaryTimer.get());

    Logger.recordOutput("OdometryTrustFactor", m_odometryTrustFactor);

    Logger.recordOutput("RobotState/NumCoralScoredAuto", m_numCoralScoredAuto);

    Logger.recordOutput("PeriodicTime/RobotState", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void coralIntakingPeriodic() {
    // when we have a game piece don't runm
    if (m_manipulator.gamePieceInFunnel() || m_manipulator.fullyIndexed()) {
      if (!(DriverStation.isAutonomous() || m_autoTestingMode)) {
        m_drive.updateProfile(DriveProfiles.kDefault);
      }
    }
    if (m_manipulator.hasGamePiece()) {
      if (!m_gamepieceUpdate) {
        m_gamepieceUpdate = true;
        m_led.gamepiece();
      }
      m_indexer.updateState(IndexerState.kIdle);
    }
    // wait until the manipulator is in position before we intake
    else if (m_manipulator.atSetpoint() && m_elevator.atSetpoint()) {
      m_indexer.updateState(IndexerState.kIndexing);
    } else {
      m_indexer.updateState(IndexerState.kIdle);
    }
  }

  public void coralOuttakingPeriodic() {
    if (m_timer.hasElapsed(ManipulatorConstants.kCoralOuttakeTimeout.get())) {
      // after we release the coral go back to stow
      if (!m_autoTestingMode && DriverStation.isAutonomous()) {
        if (m_isBargeAuto) {
          algaeDescore();
        } else {
          updateRobotAction(RobotAction.kAutoCoralIntaking);
        }
      } else {
        setDefaultAction();
      }
    }
    if (m_manipulator.hasGamePiece() || !m_timer.isRunning()) {
      m_timer.restart();
    }
  }

  public void algaeIntakingOuttakingPeriodic() {
    // we could be intaking or outtaking here
    // if we're outtaking we don't want to go to hold
  }

  public void autoScorePeriodic() {
    m_setpoint = SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight, true);
    // only set the setpoints if they're different so we don't reset the motion profile or drive pid
    if (!EqualsUtil.GeomExtensions.epsilonEquals(m_setpoint.drivePose(), m_drive.getTargetPose())) {
      m_drive.setTargetPose(m_setpoint.drivePose());
    }

    Logger.recordOutput("AutoScore/BranchIndex", m_desiredBranchIndex);
    Logger.recordOutput("AutoScore/ReefHeight", m_desiredReefHeight);
    Logger.recordOutput(
        "AutoScore/DistanceToSetpoint",
        Units.metersToInches(
            m_drive
                .getPose()
                .getTranslation()
                .getDistance(m_setpoint.drivePose().getTranslation())));

    // we don't want to deploy elevator or manipulator until we're close to the setpoint
    if (Math.abs(
            m_drive.getPose().getTranslation().getDistance(m_setpoint.drivePose().getTranslation()))
        < Units.inchesToMeters(DriveConstants.kAutoscoreDeployDistance.get())) {
      Logger.recordOutput("AutoScore/WithinDriveTolerance", Timer.getFPGATimestamp());
      if (!EqualsUtil.epsilonEquals(m_setpoint.elevatorHeight(), m_elevator.getDesiredHeight())) {
        m_elevator.setDesiredHeight(m_setpoint.elevatorHeight());
      }
      // don't deploy manipulator until we're at the elevator setpoint
      if (m_elevator.atSetpoint(10.0)) {
        if (!EqualsUtil.epsilonEquals(
            m_setpoint.manipulatorAngle().getRadians(),
            m_manipulator.getDesiredWristAngle().getRadians())) {
          m_manipulator.setDesiredWristAngle(m_setpoint.manipulatorAngle());
        }
      } else {
        m_manipulator.setDesiredWristAngle(
            Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
      }
    } else {
      m_elevator.setDesiredHeight(ElevatorConstants.kDriveUpHeight.get());
    }

    // everything must be within tolerance to run the rollers
    Logger.recordOutput("Auto/ElevRight", m_elevator.atSetpoint());
    Logger.recordOutput("Auto/ManipRight", m_manipulator.atSetpoint(m_setpoint.manipulatorAngle()));
    Logger.recordOutput(
        "Auto/DriveRight",
        m_drive.getPose().getTranslation().getDistance(m_setpoint.drivePose().getTranslation())
            < Units.inchesToMeters(
                m_desiredReefHeight == ReefHeight.L1
                    ? DriveConstants.kAutoscoreL1OuttakeDistance.get()
                    : DriveConstants.kAutoscoreOuttakeDistance.get()));
    Logger.recordOutput(
        "Auto/DriveSlow", m_drive.getMaxWheelSpeed() < DriveConstants.kAutoscoreWheelSpeed.get());
    Logger.recordOutput(
        "Auto/DriveDistance",
        Units.metersToInches(
            m_drive
                .getPose()
                .getTranslation()
                .getDistance(m_setpoint.drivePose().getTranslation())));
    Logger.recordOutput("Auto/SetpointAngle", m_setpoint.manipulatorAngle().getDegrees());
    if (m_drive.getPose().getTranslation().getDistance(m_setpoint.drivePose().getTranslation())
            < Units.inchesToMeters(
                m_desiredReefHeight == ReefHeight.L1
                    ? DriveConstants.kAutoscoreL1OuttakeDistance.get()
                    : DriveConstants.kAutoscoreOuttakeDistance.get())
        && m_elevator.atSetpoint()
        && m_manipulator.atSetpoint(m_setpoint.manipulatorAngle())
        && m_drive.getMaxWheelSpeed() < DriveConstants.kAutoscoreWheelSpeed.get()) {
      updateRobotAction(RobotAction.kCoralOuttaking);
    }
  }

  public void autoAutoScorePeriodic() {
    if (!m_secondaryTimer.isRunning()) {
      m_secondaryTimer.restart();
    }

    m_setpoint = SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight, true);
    // only set the setpoints if they're different so we don't reset the motion profile or drive pid

    // Logger.recordOutput(
    //     "NewPoses",
    //     new Pose2d[] {
    //       new Pose2d(12.92, 1.56, Rotation2d.fromDegrees(90)),
    //       new Pose2d(12.92, 6.39, Rotation2d.fromDegrees(270)),
    //       new Pose2d(4.3519, 6.39, Rotation2d.fromDegrees(270)),
    //       new Pose2d(4.3519, 1.56, Rotation2d.fromDegrees(90)),
    //     });
    // Logger.recordOutput(
    //     "OldPoses",
    //     new Pose2d[] {
    //       new Pose2d(12.92, 1.96, Rotation2d.fromDegrees(90)),
    //       new Pose2d(12.92, 5.99, Rotation2d.fromDegrees(270)),
    //       new Pose2d(4.3519, 5.99, Rotation2d.fromDegrees(270)),
    //       new Pose2d(4.3519, 1.96, Rotation2d.fromDegrees(90)),
    //     });
    if (m_isCitrusAuto && m_numCoralScoredAuto >= 3 && m_secondaryTimer.get() < 0.75) {
      if (AllianceFlipUtil.shouldFlip()) {
        // we are on red
        if (m_autoLeft) {
          m_drive.setTargetPose(new Pose2d(12.92, 1.56, Rotation2d.fromDegrees(90)));
        } else {
          m_drive.setTargetPose(new Pose2d(12.92, 6.39, Rotation2d.fromDegrees(270)));
        }
      } else {
        // we are on blue
        if (m_autoLeft) {
          m_drive.setTargetPose(new Pose2d(4.3519, 6.39, Rotation2d.fromDegrees(270)));
        } else {
          m_drive.setTargetPose(new Pose2d(4.3519, 1.56, Rotation2d.fromDegrees(90)));
        }
      }
    } else if (m_drive.getTargetPose() == null
        || !EqualsUtil.GeomExtensions.epsilonEquals(
            m_setpoint.drivePose(), m_drive.getTargetPose())) {
      m_drive.setTargetPose(m_setpoint.drivePose());
    }

    Logger.recordOutput("AutoScore/DriveTarget", m_drive.getTargetPose());
    Logger.recordOutput("AutoScore/DrivePose", m_setpoint.drivePose());

    Logger.recordOutput("AutoScore/BranchIndex", m_desiredBranchIndex);
    Logger.recordOutput("AutoScore/ReefHeight", m_desiredReefHeight);
    Logger.recordOutput(
        "AutoScore/DistanceToSetpoint",
        Units.metersToInches(
            m_drive
                .getPose()
                .getTranslation()
                .getDistance(m_setpoint.drivePose().getTranslation())));

    if (!m_manipulator.fullyIndexed()) {
      // we need to wait for the game piece to index
      if (!m_timer.isRunning()) {
        m_timer.restart();
      }

      if (m_manipulator.getCurrentState() != ManipulatorState.kIndexing) {
        m_manipulator.updateState(ManipulatorState.kIntaking);
      }
      m_elevator.updateState(ElevatorState.kIntaking);

      coralIntakingPeriodic();

      if (!m_isBargeAuto && m_timer.hasElapsed(2.5)) {
        // we give up and accept that we don't have a game piece
        m_numCoralScoredAuto++;
        updateRobotAction(RobotAction.kAutoCoralIntaking);
      }
      return;
    } else if (m_manipulator.rollerWithinTolerance()) {
      // } else {
      // we are fully in the manipulator
      m_manipulator.updateState(ManipulatorState.kScoring);
      m_elevator.updateState(ElevatorState.kScoring);
    } else if (m_manipulator.getCurrentState() == ManipulatorState.kIndexing
        && Math.abs(m_manipulator.getRollerError())
            < ManipulatorConstants.kRollerFeedErrorTolerance.get()) {
      if (!m_tertiaryTimer.isRunning()) {
        m_tertiaryTimer.restart();
      }

      if (m_tertiaryTimer.get() > 0.1) {
        m_manipulator.updateState(ManipulatorState.kScoring);
        m_elevator.updateState(ElevatorState.kScoring);
        Logger.recordOutput("Autoscore/SkippedIndexing", Timer.getFPGATimestamp());
      }
    }

    // we don't want to deploy elevator or manipulator until we're close to the setpoint
    if (Math.abs(
            m_drive.getPose().getTranslation().getDistance(m_setpoint.drivePose().getTranslation()))
        < Units.inchesToMeters(DriveConstants.kAutoscoreDeployDistance.get())) {
      Logger.recordOutput("AutoScore/WithinDriveTolerance", Timer.getFPGATimestamp());
      if (!EqualsUtil.epsilonEquals(m_setpoint.elevatorHeight(), m_elevator.getDesiredHeight())) {
        m_elevator.setDesiredHeight(m_setpoint.elevatorHeight());
      }
      // don't deploy manipulator until we're at the elevator setpoint
      if (m_elevator.atSetpoint(10.0)) {
        if (!EqualsUtil.epsilonEquals(
            m_setpoint.manipulatorAngle().getRadians(),
            m_manipulator.getDesiredWristAngle().getRadians())) {
          m_manipulator.setDesiredWristAngle(m_setpoint.manipulatorAngle());
        }
      } else {
        m_manipulator.setDesiredWristAngle(
            Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
      }
    } else {
      // as we drive up go to a height still within the first stage so don't increase center of mass
      m_elevator.setDesiredHeight(ElevatorConstants.kDriveUpHeight.get());
    }

    // everything must be within tolerance to run the rollers
    Logger.recordOutput("Auto/ElevRight", m_elevator.atSetpoint());
    Logger.recordOutput("Auto/ManipRight", m_manipulator.atSetpoint(m_setpoint.manipulatorAngle()));
    Logger.recordOutput(
        "Auto/DriveRight",
        m_drive.getPose().getTranslation().getDistance(m_setpoint.drivePose().getTranslation())
            < Units.inchesToMeters(
                m_desiredReefHeight == ReefHeight.L1
                    ? DriveConstants.kAutoscoreL1OuttakeDistance.get()
                    : DriveConstants.kAutoscoreOuttakeDistance.get()));
    Logger.recordOutput(
        "Auto/DriveSlow", m_drive.getMaxWheelSpeed() < DriveConstants.kAutoscoreWheelSpeed.get());
    Logger.recordOutput(
        "Auto/HeadingRight",
        EqualsUtil.epsilonEquals(
            m_drive.getRotation().getDegrees(),
            m_setpoint.drivePose().getRotation().getDegrees(),
            DriveConstants.kAutoscoreHeadingTolerance));
    Logger.recordOutput(
        "Auto/DriveDistance",
        Units.metersToInches(
            m_drive
                .getPose()
                .getTranslation()
                .getDistance(m_setpoint.drivePose().getTranslation())));
    Logger.recordOutput("Auto/SetpointAngle", m_setpoint.manipulatorAngle().getDegrees());
    if (m_drive.getPose().getTranslation().getDistance(m_setpoint.drivePose().getTranslation())
            < Units.inchesToMeters(
                m_desiredReefHeight == ReefHeight.L1
                    ? DriveConstants.kAutoscoreL1OuttakeDistance.get()
                    : DriveConstants.kAutoscoreOuttakeDistance.get())
        && m_elevator.atSetpoint()
        && m_manipulator.atSetpoint(m_setpoint.manipulatorAngle())
        && m_drive.getMaxWheelSpeed() < DriveConstants.kAutoscoreWheelSpeed.get()) {
      m_numCoralScoredAuto++;
      updateRobotAction(RobotAction.kAutoCoralOuttaking);
    }
  }

  private boolean m_autoLeft = true;
  private Timer m_loaderStationTimer = new Timer();
  private int m_numCoralScoredAuto = 0;
  private int m_numAlgaeScoredAuto = 0;

  public void autoCoralIntakingPeriodic() {
    // TODO: make auto setpoint generator to make this cleaner
    if (!m_timer.isRunning()) {
      m_timer.restart();
    }

    if (m_timer.get() < 0.25 && m_numCoralScoredAuto < 2) {
      if (AllianceFlipUtil.shouldFlip()) {
        // we are on red
        if (m_autoLeft) {
          m_drive.setTargetPose(new Pose2d(12.92, 1.96, m_drive.getRotation()));
        } else {
          m_drive.setTargetPose(new Pose2d(12.92, 5.99, m_drive.getRotation()));
        }
      } else {
        // we are on blue
        if (m_autoLeft) {
          m_drive.setTargetPose(new Pose2d(4.3519, 5.99, m_drive.getRotation()));
        } else {
          m_drive.setTargetPose(new Pose2d(4.3519, 1.96, m_drive.getRotation()));
        }
      }
    } else {
      if (AllianceFlipUtil.shouldFlip()) {
        // we are on red
        if (m_autoLeft) {
          m_drive.setTargetPose(new Pose2d(15.92385, 0.9075, Rotation2d.fromDegrees(126)));
        } else {
          // I HATE MATH
          m_drive.setTargetPose(
              new Pose2d(
                  15.92385, 7.1825, Rotation2d.fromDegrees(54).plus(Rotation2d.fromDegrees(180))));
        }
      } else {
        // we are on blue
        if (m_autoLeft) {
          m_drive.setTargetPose(new Pose2d(1.6244, 7.1825, Rotation2d.fromDegrees(-54)));
        } else {
          m_drive.setTargetPose(
              new Pose2d(
                  1.6244, 0.9075, Rotation2d.fromDegrees(-126).plus(Rotation2d.fromDegrees(180))));
        }
      }
    }

    if (RobotBase.isReal() && DriverStation.getMatchTime() < 1.0) {
      var targetPose = m_drive.getTargetPose();
      var newTarget = new Pose2d(targetPose.getTranslation(), m_drive.getRotation());
      m_drive.setTargetPose(newTarget);
    }

    coralIntakingPeriodic();

    if (!m_autoTestingMode && m_manipulator.gamePieceInFunnel()) {
      if (m_numCoralScoredAuto < 2) {
        if (m_autoLeft) {
          setReefIndexLeft();
        } else {
          setReefIndexRight();
        }
      } else if (m_numCoralScoredAuto < 3) {
        if (m_autoLeft) {
          setReefIndexRight();
        } else {
          setReefIndexLeft();
        }
      } else {
        // I. Hate. Math.
        Pose2d pose;
        if (m_isCitrusAuto) {
          if (m_autoLeft) {
            pose =
                new Pose2d(
                        AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                        Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? -120 : 60))
                    .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
          } else {
            pose =
                new Pose2d(
                        AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                        Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 120 : -60))
                    .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
          }
        } else {
          pose =
              new Pose2d(
                      AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                      AllianceFlipUtil.apply(Rotation2d.fromDegrees(180)))
                  .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
        }
        if (m_autoLeft) {
          setReefIndexLeft(pose);
        } else {
          setReefIndexRight(pose);
        }
      }
      updateRobotAction(RobotAction.kAutoAutoScore);
      return;
    }

    Logger.recordOutput(
        "AutoCoralIntake/DriveRight",
        m_drive.driveToPointWithinTolerance(Inches.of(3.0), Degrees.of(10)));
    if (!m_autoTestingMode && m_drive.driveToPointWithinTolerance(Inches.of(3.0), Degrees.of(10))) {
      if (!m_loaderStationTimer.isRunning()) {
        m_loaderStationTimer.restart();
      } else if (m_loaderStationTimer.hasElapsed(DriveConstants.kLoaderStationTimeout.get())) {
        if (m_numCoralScoredAuto < 2) {
          if (m_autoLeft) {
            setReefIndexLeft();
          } else {
            setReefIndexRight();
          }
        } else if (m_numCoralScoredAuto < 3) {
          if (m_autoLeft) {
            setReefIndexRight();
          } else {
            setReefIndexLeft();
          }
        } else {
          // I. Hate. Math. pt2
          Pose2d pose;
          if (m_isCitrusAuto) {
            if (m_autoLeft) {
              pose =
                  new Pose2d(
                          AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                          Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? -120 : 60))
                      .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
            } else {
              pose =
                  new Pose2d(
                          AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                          Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 120 : -60))
                      .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
            }
          } else {
            pose =
                new Pose2d(
                        AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                        AllianceFlipUtil.apply(Rotation2d.fromDegrees(180)))
                    .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
          }
          if (m_autoLeft) {
            setReefIndexLeft(pose);
          } else {
            setReefIndexRight(pose);
          }
        }
        updateRobotAction(RobotAction.kAutoAutoScore);
      }
      Logger.recordOutput("AutoCoralIntake/LoaderStationTimer", m_loaderStationTimer.get());
    }
  }

  public void manualScorePeriodic() {
    m_setpoint = SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight, false);

    Logger.recordOutput("ManualScore/BranchIndex", m_desiredBranchIndex);
    Logger.recordOutput("ManualScore/ReefHeight", m_desiredReefHeight);

    // only set the setpoints if they're different so we don't reset the motion profile
    if (!EqualsUtil.epsilonEquals(m_setpoint.elevatorHeight(), m_elevator.getDesiredHeight())) {
      m_elevator.setDesiredHeight(m_setpoint.elevatorHeight());
    }
    // don't deploy manipulator until we're at the elevator setpoint
    if (m_elevator.atSetpoint(10.0)) {
      if (!EqualsUtil.epsilonEquals(
          m_setpoint.manipulatorAngle().getRadians(),
          m_manipulator.getDesiredWristAngle().getRadians())) {
        m_manipulator.setDesiredWristAngle(m_setpoint.manipulatorAngle());
      }
    } else {
      m_manipulator.setDesiredWristAngle(
          Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
    }
    // don't worry about drive in manual score
  }

  public void driveToProcessorPeriodic() {
    if (m_drive.driveToPointWithinTolerance()) {
      updateRobotAction(RobotAction.kProcessorOuttake);
    }

    Pose2d processorPose = FieldConstants.Processor.kCenterFace;
    processorPose =
        AllianceFlipUtil.shouldFlip()
            ? processorPose.rotateAround(
                new Translation2d(
                    FieldConstants.kFieldLength / 2.0, FieldConstants.kFieldWidth / 2.0),
                Rotation2d.fromDegrees(180))
            : processorPose;
    processorPose =
        processorPose.transformBy(
            new Transform2d(
                Meters.of(DriveConstants.kTrackWidthX / 2).plus(Inches.of(10.0)),
                Inches.zero(),
                Rotation2d.fromDegrees(180)));
    if (!EqualsUtil.GeomExtensions.epsilonEquals(processorPose, m_drive.getTargetPose())) {
      m_drive.setTargetPose(processorPose);
    }
  }

  public void bargeScorePeriodic() {
    if (!m_hasRunASingleCycle) {
      m_hasRunASingleCycle = true;
      return;
    }

    if (m_elevator.atSetpoint(ElevatorConstants.kBargeThrowHeight.get())) {
      m_manipulator.runRollerBargeScoring();
    }
  }

  public void bargeAutoScorePeriodic() {
    // only autoshoot in auto
    if ((DriverStation.isAutonomous() || m_autoTestingMode)
        && m_drive.driveToPointWithinTolerance(
            Inches.of(DriveConstants.kBargeScoreThrowDistance.get()), Degrees.of(10.0))) {
      m_elevator.updateState(ElevatorState.kBargeScore);

      if (!m_hasRunASingleCycle) {
        m_hasRunASingleCycle = true;
        return;
      }

      if (m_elevator.atSetpoint(ElevatorConstants.kBargeThrowHeight.get())) {
        if (!m_timer.isRunning()) {
          m_timer.restart();
        }

        m_manipulator.runRollerBargeScoring();
        if ((DriverStation.isAutonomous() || m_autoTestingMode) && m_timer.hasElapsed(0.4)) {
          m_numAlgaeScoredAuto++;
          m_manipulator.updateState(ManipulatorState.kStow);
          algaeDescore();
        }
      }
    }

    if (m_elevator.getCurrentState() == ElevatorState.kBargeScore) {
      if (!m_hasRunASingleCycle) {
        m_hasRunASingleCycle = true;
        return;
      }

      if (m_elevator.atSetpoint(ElevatorConstants.kBargeThrowHeight.get())) {
        if (!m_timer.isRunning()) {
          m_timer.restart();
        }

        m_manipulator.runRollerBargeScoring();
      }
    }
  }

  public void lollipopIntakePeriodic() {
    // var heading = SetpointGenerator.generateLollipopAngle(m_drive.getPose().getTranslation());

    // m_drive.setDesiredHeading(heading);
  }

  public void coralOTBPeriodic() {}

  public void coralOTBL1Periodic() {}

  // this is a hack but trust
  private boolean m_hasRunASingleCycle = false;

  private boolean m_hasMovedOnFromTheIntermediateToTheDescore = false;

  public void algaeDescoringPeriodic() {

    var sequence =
        (DriverStation.isAutonomous() || m_autoTestingMode)
            ? kAlgaeDescoreSequenceAuto
            : kAlgaeDescoreSequence;

    if (!m_timer.isRunning()) {
      m_timer.restart();
    }

    if (getUsingVision()) {
      if (!m_hasMovedOnFromTheIntermediateToTheDescore
          && (!m_hasRunASingleCycle
              || ((DriverStation.isAutonomous() || m_autoTestingMode)
                  && m_profiles.getCurrentProfile() == RobotAction.kAlgaeDescoringInitial
                  && m_timer.get() < 10.0
                  && !m_drive.driveToPointWithinTolerance(Inches.of(24.0), Degrees.of(10.0))))) {

        m_drive.setTargetPose(
            SetpointGenerator.generateAlgae(m_desiredAlgaeIndex)
                .transformBy(
                    new Transform2d(
                        Inches.of(m_numAlgaeScoredAuto < 2 ? -30 : -40),
                        Inches.zero(),
                        new Rotation2d())));

      } else if (m_profiles.getCurrentProfile() == RobotAction.kAlgaeDescoringDriveAway) {
        m_drive.setTargetPose(SetpointGenerator.generateAlgaeFinal(m_desiredAlgaeIndex));
      } else {
        m_hasMovedOnFromTheIntermediateToTheDescore = true;
        m_drive.setTargetPose(SetpointGenerator.generateAlgae(m_desiredAlgaeIndex));
      }
    }

    // each step in the sequence is identical in terms of whether to advance to the next step
    int index = sequence.indexOf(m_profiles.getCurrentProfile());
    Logger.recordOutput("AlgaeDescore/ElevatorAtSetpoint", m_elevator.atSetpoint());
    Logger.recordOutput("AlgaeDescore/ManipulatorAtSetpoint", m_manipulator.atSetpoint());
    Logger.recordOutput(
        "AlgaeDescore/DriveWithinTolerance",
        m_drive.driveToPointWithinTolerance(
            Meters.of((DriverStation.isAutonomous() || m_autoTestingMode) ? 2.0 : 0.1), null));
    Logger.recordOutput("AlgaeDescore/Index", index);

    if (m_elevator.atSetpoint()
        && (m_manipulator.atSetpoint() || (DriverStation.isAutonomous() || m_autoTestingMode))
        && m_drive.driveToPointWithinTolerance(
            Meters.of((DriverStation.isAutonomous() || m_autoTestingMode) ? 2.0 : 0.1), null)
        && m_hasRunASingleCycle) {
      if (index < sequence.size() - 2) {
        Logger.recordOutput("AlgaeDescore/TransitionTime", Timer.getFPGATimestamp());
        updateRobotAction(sequence.get(index + 1));
      } else if (index == sequence.size() - 1) {
        // if we're on the last step then we want to go back to stow
        if ((DriverStation.isAutonomous() || m_autoTestingMode)) {
          setBargeScoreRight();
          updateRobotAction(RobotAction.kBargeAutoScore);
          return;
        } else {
          m_manipulator.updateState(ManipulatorState.kAlgaeHold);
        }
        setDefaultAction();
      } else if ((DriverStation.isAutonomous() || m_autoTestingMode)
          && index == sequence.size() - 2
          && m_timer.hasElapsed(m_numAlgaeScoredAuto < 1 ? 1.0 : 1.0)) {
        // if we in auto then wait enough time to descore
        updateRobotAction(sequence.get(index + 1));
      }
      // if we're on the second to last step then don't advance and wait for controller input
    }

    if (!m_hasRunASingleCycle) {
      m_hasRunASingleCycle = true;
    }
  }

  public void updateRobotAction(RobotAction newAction) {
    DriveProfiles newDriveProfiles = m_drive.getDefaultProfile();
    IntakeState newIntakeState = m_intake.getStowOrHold();
    IndexerState newIndexerState = IndexerState.kIdle;
    ElevatorState newElevatorState = ElevatorState.kStow;
    ManipulatorState newManipulatorState = m_manipulator.getStowOrHold();

    switch (newAction) {
      case kAlgaeIntakingOuttaking:
        newIntakeState = m_intake.getIntakeOrOuttake();

        break;

      case kManualScore:
        newElevatorState = ElevatorState.kScoring;
        newManipulatorState = ManipulatorState.kScoring;

        // guys don't use fallthrough it's not worth it
        break;

      case kAutoScore:
      case kAutoAutoScore:
        newDriveProfiles = DriveProfiles.kDriveToPoint;
        m_drive.setTargetPose(m_setpoint.drivePose());
        newElevatorState = ElevatorState.kScoring;
        newManipulatorState = ManipulatorState.kScoring;
        break;

      case kAutoCoralIntaking:
        newDriveProfiles = DriveProfiles.kDriveToPoint;
        newIntakeState = IntakeState.kFunnelIntaking;
        newElevatorState = ElevatorState.kIntaking;
        newManipulatorState = ManipulatorState.kIntaking;
        break;

      case kDriveToProcessor:
        newDriveProfiles = DriveProfiles.kDriveToPoint;

        break;

      case kCoralIntaking:
        newIntakeState = IntakeState.kFunnelIntaking;
        newElevatorState = ElevatorState.kIntaking;
        newManipulatorState = ManipulatorState.kIntaking;
        if (getUsingVision()) {
          newDriveProfiles = DriveProfiles.kMeshedUserControls;
        }

        break;

      case kAutoDefault:
      case kTeleopDefault:
        break;

      case kAutoCoralOuttaking:
      case kCoralOuttaking:
        // don't change any other states since we want everything to stay in place
        newDriveProfiles = m_drive.getCurrentProfile();
        // newDriveProfiles = DriveProfiles.kStop;
        newIntakeState = m_intake.getCurrentState();
        newIndexerState = m_indexer.getCurrentState();
        newElevatorState = m_elevator.getCurrentState();
        newManipulatorState = m_manipulator.getCurrentState();

        m_manipulator.runRollerScoring();
        break;

      case kBargeOuttaking:
        // don't change any other states since we want everything to stay in place
        // name is kinda confusing but we would still be in the algae descore sequence on the
        // manipulator
        newDriveProfiles = m_drive.getCurrentProfile();
        newIntakeState = m_intake.getCurrentState();
        newIndexerState = m_indexer.getCurrentState();
        newElevatorState = m_elevator.getCurrentState();
        newManipulatorState = m_manipulator.getCurrentState();

        m_manipulator.runRollerAlgaeDescoring();
        break;

      case kBargeScore:
        newElevatorState = ElevatorState.kBargeScore;
        newManipulatorState = ManipulatorState.kAlgaeHold;

        break;

      case kBargeAutoScore:
        if ((DriverStation.isAutonomous() || m_autoTestingMode)) {
          newDriveProfiles = DriveProfiles.kDriveToPoint;
          m_drive.setTargetPose(
              m_bargeLeftCage
                  ? SetpointGenerator.generateBargeLeft()
                  : SetpointGenerator.generateBargeRight());
        } else {
          newDriveProfiles = DriveProfiles.kMeshedUserControls;
        }
        newManipulatorState = ManipulatorState.kAlgaeHold;

        break;

      case kProcessorOuttake:
        newManipulatorState = ManipulatorState.kAlgaeOuttake;

        break;

      case kCoralEject:
        newManipulatorState = ManipulatorState.kCoralEject;
        newElevatorState = ElevatorState.kIntaking;
        newIndexerState = IndexerState.kCoralEject;
        newIntakeState = IntakeState.kCoralEject;

        break;

      case kLollipopIntake:
        newManipulatorState = ManipulatorState.kAlgaeHold;
        newElevatorState = ElevatorState.kLollipopIntake;

        break;

      case kCoralOTB:
        newIntakeState = IntakeState.kIntaking;
        newElevatorState = ElevatorState.kIntaking;
        newManipulatorState = ManipulatorState.kIntaking;

        break;

      case kCoralOTBL1:
        newIntakeState = IntakeState.kOuttaking;

        break;

      case kAlgaeDescoringInitial:
        if (getUsingVision()) {
          newDriveProfiles = DriveProfiles.kDriveToPoint;
          if ((DriverStation.isAutonomous() || m_autoTestingMode)) {
            if (m_numAlgaeScoredAuto < 1) {
              m_desiredAlgaeIndex = SetpointGenerator.getAlgaeIndex(m_drive.getPose());
            } else if (m_numAlgaeScoredAuto < 2) {
              m_desiredAlgaeIndex =
                  SetpointGenerator.getAlgaeIndex(
                      new Pose2d(
                              AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                              AllianceFlipUtil.shouldFlip()
                                  ? Rotation2d.fromDegrees(-120)
                                  : Rotation2d.fromDegrees(60))
                          .transformBy(new Transform2d(4.0, 0.0, new Rotation2d())));
            } else {
              m_desiredAlgaeIndex =
                  SetpointGenerator.getAlgaeIndex(
                      new Pose2d(
                              AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                              AllianceFlipUtil.shouldFlip()
                                  ? Rotation2d.fromDegrees(120)
                                  : Rotation2d.fromDegrees(-60))
                          .transformBy(new Transform2d(4.0, 0.0, new Rotation2d())));
            }
          } else {
            m_desiredAlgaeIndex = SetpointGenerator.getAlgaeIndex(m_drive.getPose());
          }
        }
        if ((DriverStation.isAutonomous() || m_autoTestingMode)) {
          newElevatorState = ElevatorState.kStow;
        } else {
          newElevatorState = ElevatorState.kAlgaeDescoringInitial;
        }

        break;

      case kAlgaeDescoringDeployManipulator:
        if (getUsingVision()) {
          newDriveProfiles = DriveProfiles.kDriveToPoint;
          m_desiredAlgaeIndex = SetpointGenerator.getAlgaeIndex(m_drive.getPose());
        }
        newElevatorState = ElevatorState.kAlgaeDescoringInitial;
        newManipulatorState = ManipulatorState.kAlgaeDescoring;

        break;

      case kAlgaeDescoringMoveUp:
        if (getUsingVision()) {
          newDriveProfiles = DriveProfiles.kDriveToPoint;
          m_desiredAlgaeIndex = SetpointGenerator.getAlgaeIndex(m_drive.getPose());
        }
        newElevatorState = ElevatorState.kAlgaeDescoringFinal;
        newManipulatorState = ManipulatorState.kAlgaeDescoring;

        break;

      case kAlgaeDescoringDriveAway:
        if (getUsingVision()) {
          newDriveProfiles = DriveProfiles.kDriveToPoint;
          m_desiredAlgaeIndex = SetpointGenerator.getAlgaeIndex(m_drive.getPose());
          m_drive.setTargetPose(SetpointGenerator.generateAlgaeFinal(m_desiredAlgaeIndex));
        }
        newElevatorState = ElevatorState.kAlgaeDescoringFinal;
        newManipulatorState = ManipulatorState.kAlgaeHold;

        break;

      case kAlgaeDescoringFinal:
        if (getUsingVision()) {
          newDriveProfiles = DriveProfiles.kDriveToPoint;
          m_desiredAlgaeIndex = SetpointGenerator.getAlgaeIndex(m_drive.getPose());
        }
        newManipulatorState = ManipulatorState.kAlgaeHold;

        break;
    }

    m_profiles.setCurrentProfile(newAction);

    if (m_drive.getCurrentProfile() != newDriveProfiles) {
      m_drive.updateProfile(newDriveProfiles);
    }

    if (m_intake.getCurrentState() != newIntakeState) {
      m_intake.updateState(newIntakeState);
    }

    if (m_indexer.getCurrentState() != newIndexerState) {
      m_indexer.updateState(newIndexerState);
    }

    if (m_elevator.getCurrentState() != newElevatorState) {
      m_elevator.updateState(newElevatorState);
    }

    if (m_manipulator.getCurrentState() != newManipulatorState) {
      m_manipulator.updateState(newManipulatorState);
    }

    // stop the timer so a previous action doesn't affect the new one
    m_timer.stop();
    m_timer.reset();

    m_secondaryTimer.stop();
    m_secondaryTimer.reset();

    m_tertiaryTimer.stop();
    m_tertiaryTimer.reset();

    m_loaderStationTimer.stop();
    m_loaderStationTimer.reset();

    m_hasRunASingleCycle = false;

    m_hasMovedOnFromTheIntermediateToTheDescore = false;

    m_gamepieceUpdate = false;

    m_aprilTagVision.resetAutoAutoScoreMeasurements();

    // reset the scoring angles
    if (newAction != RobotAction.kCoralOuttaking && newAction != RobotAction.kAutoCoralOuttaking) {
      m_manipulator.setDesiredWristAngle(
          Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
      m_elevator.setDesiredHeight(ElevatorConstants.kStowHeight.get());
    }
  }

  public RobotAction getCurrentAction() {
    return m_profiles.getCurrentProfile();
  }

  public void setDefaultAction() {
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      updateRobotAction(RobotAction.kAutoDefault);
    } else {
      updateRobotAction(RobotAction.kTeleopDefault);
    }
  }

  public void algaeDescore() {
    updateRobotAction(RobotAction.kAlgaeDescoringInitial);
  }

  public void manageAutoScoreButton() {
    // one button will go to different locations on the field based on the current location
    // if we're at the processor then kDriveToProcessor
    // if we're near the barge then kBargeScore
    // and if we're near the reef then kAutoScore
    // if (SetpointGenerator.isNearProcessor(m_drive.getPose())) {
    //   updateRobotAction(RobotAction.kDriveToProcessor);
    // } else if (SetpointGenerator.isNearBarge(m_drive.getPose())) {
    //   updateRobotAction(RobotAction.kBargeScore);
    // } else {
    updateRobotAction(RobotAction.kAutoScore);
    // }
  }

  public void manageCoralOuttakePressed() {
    // this button will score coral but also score algae if we're at the barge

    updateRobotAction(RobotAction.kCoralOuttaking);
  }

  public void manageAlgaeIntake() {
    if (m_profiles.getCurrentProfile() == RobotAction.kBargeAutoScore
        || m_profiles.getCurrentProfile() == RobotAction.kProcessorOuttake) {
      m_elevator.updateState(ElevatorState.kBargeScore);
    } else if (m_manipulator.getCurrentState() == ManipulatorState.kAlgaeHold) {
      // if we're near the processor then initiate a drive to point
      // if (getUsingVision() && SetpointGenerator.isNearProcessor(m_drive.getPose())) {
      // updateRobotAction(RobotAction.kDriveToProcessor);
      // } else {
      updateRobotAction(RobotAction.kProcessorOuttake);
      // }
    } else {
      // first press - run intake
      // second press - L1
      if (!m_otbRunThrough && m_intake.getCurrentState() == IntakeState.kCoralHold) {
        updateRobotAction(RobotAction.kCoralOTBL1);
      } else {
        updateRobotAction(RobotAction.kCoralOTB);
      }
    }
  }

  public void manageAlgaeIntakeRelease() {
    if (m_profiles.getCurrentProfile() == RobotAction.kBargeAutoScore
        || m_profiles.getCurrentProfile() == RobotAction.kProcessorOuttake) {
      setDefaultAction();
    }
    if (m_otbRunThrough) {
      m_intake.updateState(IntakeState.kCoralRunThrough);
    } else {
      // if we're L1 then stow
      // otherwise hold
      if (m_profiles.getCurrentProfile() == RobotAction.kCoralOTBL1) {
        m_intake.updateState(IntakeState.kStow);
      } else if (m_profiles.getCurrentProfile() == RobotAction.kCoralOTB) {
        m_intake.updateState(IntakeState.kCoralHold);
      }
      setDefaultAction();
    }
  }

  public void manageAlgaeDescoreRelease() {
    if (m_profiles.getCurrentProfile()
        == kAlgaeDescoreSequence.get(kAlgaeDescoreSequence.size() - 2)) {
      m_manipulator.updateState(ManipulatorState.kAlgaeHold);
    }
    setDefaultAction();
  }

  public void manageIntakeHasGamePiece() {
    if (m_otbRunThrough) {
      m_intake.updateState(IntakeState.kCoralRunThrough);
    } else {
      // if we're L1 then stow
      // otherwise hold
      if (m_profiles.getCurrentProfile() == RobotAction.kCoralOTBL1) {
        return;
      } else if (m_profiles.getCurrentProfile() == RobotAction.kCoralOTB) {
        m_intake.updateState(IntakeState.kCoralHold);
      }
      setDefaultAction();
    }
  }

  public void onEnable() {
    m_drive.setBrake();

    setDefaultAction();

    m_numCoralScoredAuto = 0;
    m_numAlgaeScoredAuto = 0;
  }

  public void onDisable() {
    // if (m_profiles.getCurrentProfile() == RobotAction.kAutoCoralIntaking) {
    //   m_drive.setDriveCoast();
    // }
  }

  public void addTimestampedVisionObservations(List<TimestampedVisionUpdate> observations) {
    if (!m_usingVision) {
      return;
    }
    if (observations.size() == 0) {
      m_odometryTrustFactor -= AprilTagVisionConstants.kOdometryTrustFactorNoVision;
    } else {
      for (var observation : observations) {
        m_odometryTrustFactor +=
            AprilTagVisionConstants.kOdometryTrustFactorVisionScalar
                / observation.stdDevs().get(0, 0);
      }
    }
    m_odometryTrustFactor = MathUtil.clamp(m_odometryTrustFactor, 0.0, 1.0);
    m_drive.addTimestampedVisionObservations(observations);
  }

  // public void addTimestampedVisionObservationsCameras(
  //     List<List<TimestampedVisionUpdate>> observations) {
  //   if (!m_usingVision) {
  //     return;
  //   }

  //   m_drive.addTimestampedVisionObservationsCameras(observations);
  // }

  public void registerSlip() {
    m_odometryTrustFactor -= AprilTagVisionConstants.kOdometryTrustFactorSlip;
    m_odometryTrustFactor = MathUtil.clamp(m_odometryTrustFactor, 0.0, 1.0);
  }

  public void setDesiredReefHeight(ReefHeight height) {
    m_desiredReefHeight = height;
  }

  public void setDesiredBranchIndex(int index) {
    m_desiredBranchIndex = index;
  }

  public ReefHeight getDesiredReefHeight() {
    return m_desiredReefHeight;
  }

  public int getDesiredBranchIndex() {
    return m_desiredBranchIndex;
  }

  public boolean getBargeLeftCage() {
    return m_bargeLeftCage;
  }

  public void manageCoralOuttakeRelease() {
    // if we successfully outtake the coral we want to go back to stow
    // otherwise go back to autoscore/manual score
    if (!m_manipulator.hasGamePiece()) {
      setDefaultAction();
    } else {
      // i'm actually using revertToLastProfile
      // i never thought this day would come
      m_profiles.revertToLastProfile();
    }
  }

  public void setReefIndexLeft() {
    setReefIndexLeft(getRobotPose());
  }

  private void setReefIndexLeft(Pose2d currPose) {
    var indices = SetpointGenerator.getPossibleIndices(currPose);
    // indices are in order starting from the driver station and going clockwise
    // so the second index is the one to the left
    if (indices.getSecond() != -1) {
      m_desiredBranchIndex = indices.getSecond();
    }
  }

  public void setReefIndexRight() {
    setReefIndexRight(getRobotPose());
  }

  private void setReefIndexRight(Pose2d currPose) {
    var indices = SetpointGenerator.getPossibleIndices(currPose);
    // indices are in order starting from the driver station and going clockwise
    // so the first index is the one to the right
    if (indices.getFirst() != -1) {
      m_desiredBranchIndex = indices.getFirst();
    }
  }

  public void setBargeScoreLeft() {
    m_bargeLeftCage = true;
  }

  public void setBargeScoreRight() {
    m_bargeLeftCage = false;
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public void incrementCoralScoredAuto() {
    m_numCoralScoredAuto++;
  }

  public void decrementCoralScoredAuto() {
    m_numCoralScoredAuto--;
  }

  public ChassisSpeeds getRobotSpeeds() {
    return m_drive.getChassisSpeeds();
  }

  public void updateComponent() {
    Pose3d intakePose =
        new Pose3d( // Algae Intake
            new Translation3d(-0.31, 0, 0.19),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(-150).plus(m_intake.getRotation().getMeasure()),
                Degrees.of(180)));
    Pose3d elevatorStage2Pose =
        new Pose3d(
            new Translation3d(
                Meters.zero(),
                Meters.zero(),
                Inches.of(Math.max(0, m_elevator.getCurrHeight() - 45))),
            new Rotation3d());
    Pose3d elevatorStage3Pose =
        new Pose3d(
            new Translation3d(
                Meters.zero(),
                Meters.zero(),
                Inches.of(Math.max(0, m_elevator.getCurrHeight() - 18))),
            new Rotation3d());
    Pose3d carriagePose =
        new Pose3d(
            new Translation3d(Meters.zero(), Meters.zero(), Inches.of(m_elevator.getCurrHeight())),
            new Rotation3d());
    Pose3d manipulatorPose =
        new Pose3d( // Manipulator
            new Translation3d(
                Meters.of(0.285),
                Meters.zero(),
                Meters.of(0.203).plus(Inches.of(m_elevator.getCurrHeight()))),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(-m_manipulator.getCurrAngle().getDegrees()),
                Degrees.of(0)));
    Pose3d climbPose =
        new Pose3d( // static for now
            new Translation3d(0, -0.336, 0.405),
            new Rotation3d(Degrees.of(-90), Degrees.of(0), Degrees.of(0)));
    Logger.recordOutput(
        "FieldSimulation/FinalComponentPoses",
        new Pose3d[] {
          intakePose,
          elevatorStage2Pose,
          elevatorStage3Pose,
          carriagePose,
          manipulatorPose,
          climbPose,
        });
  }

  public void updateLED() {
    if (FullTuningConstants.kFullTuningMode) {
      m_led.updateState(LedState.kFullTuning);
      return;
    }
    if (DriverStation.isDisabled()) {
      // do nothing here because we don't want the timer to reset
      return;
    }
    if ((DriverStation.isAutonomous() || m_autoTestingMode)
        && m_profiles.getCurrentProfile() == RobotAction.kAutoAutoScore) {
      if (m_aprilTagVision.getAutoAutoScoreMeasurements() > 30) {
        m_led.updateState(LedState.kAutoscoreMeasurementsGood);
      } else {
        m_led.updateState(LedState.kAutoscoreMeasurementsBad);
      }
      return;
    }
    // if we get here then we're in teleop so we should pick based on what level is selected
    switch (m_desiredReefHeight) {
      case L1:
        m_led.updateState(LedState.kL1);
        break;
      case L2:
        m_led.updateState(LedState.kL2);
        break;
      case L3:
        m_led.updateState(LedState.kL3);
        break;
      case L4:
        m_led.updateState(LedState.kL4);
        break;
    }
  }

  public int getNumVisionGyroObservations() {
    return m_numVisionGyroObservations;
  }

  public void incrementNumVisionGyroObservations() {
    m_numVisionGyroObservations++;
  }

  private boolean m_usingVision = true;

  public boolean getUsingVision() {
    return m_usingVision;
  }

  public void toggleUsingVision() {
    m_usingVision = !m_usingVision;
    m_led.updateLEDState();
  }

  public void toggleOtbRunthrough() {
    m_otbRunThrough = !m_otbRunThrough;
    if (m_otbRunThrough && m_intake.getCurrentState() == IntakeState.kCoralHold) {
      updateRobotAction(RobotAction.kCoralOTB);
      m_intake.updateState(IntakeState.kCoralRunThrough);
    }
  }

  public boolean getOtbRunthrough() {
    return m_otbRunThrough;
  }

  public boolean manipulatorAtSetpoint() {
    return m_manipulator.atSetpoint();
  }

  public double getYawVelocity() {
    return m_drive.getYawVelocity();
  }

  public void setDrivePose(Pose2d pose) {
    m_drive.setPose(pose);
  }

  private String m_selectedAuto;

  public void setSelectedAuto(String name) {
    m_selectedAuto = name;
  }

  public void calculateDriveTargetPose() {
    setReefIndexLeft();
    setDesiredReefHeight(ReefHeight.L4);
    m_setpoint = SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight, true);
    m_drive.setTargetPose(m_setpoint.drivePose());

    // find if we're in the wrong spot for the auto we have selected
    // only if fms connected
    if (DriverStation.isFMSAttached()) {
      Pose2d expectedTarget = null;
      switch (m_selectedAuto.toLowerCase()) {
        case "3 coral left":
        case "3 coral left citrus":
          expectedTarget =
              SetpointGenerator.generate(
                      SetpointGenerator.getPossibleIndices(
                              new Pose2d(
                                      AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                                      Rotation2d.fromDegrees(
                                          AllianceFlipUtil.shouldFlip() ? -120 : 60))
                                  .transformBy(new Transform2d(3, 0, new Rotation2d())))
                          .getSecond(),
                      ReefHeight.L4,
                      true)
                  .drivePose();
          break;

        case "3 coral right":
        case "3 coral right citrus":
          expectedTarget =
              SetpointGenerator.generate(
                      SetpointGenerator.getPossibleIndices(
                              new Pose2d(
                                      AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                                      Rotation2d.fromDegrees(
                                          AllianceFlipUtil.shouldFlip() ? 120 : -60))
                                  .transformBy(new Transform2d(3, 0, new Rotation2d())))
                          .getSecond(),
                      ReefHeight.L4,
                      true)
                  .drivePose();
          break;

        case "center barge":
          expectedTarget =
              SetpointGenerator.generate(
                      SetpointGenerator.getPossibleIndices(
                              new Pose2d(
                                      AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                                      Rotation2d.fromDegrees(
                                          AllianceFlipUtil.shouldFlip() ? 180 : 0))
                                  .transformBy(new Transform2d(3, 0, new Rotation2d())))
                          .getSecond(),
                      ReefHeight.L4,
                      true)
                  .drivePose();
          break;
      }

      Logger.recordOutput("PreAuto/ExpectedTarget", expectedTarget);
      Logger.recordOutput("PreAuto/CurrentTarget", m_drive.getTargetPose());
      if (expectedTarget != null
          && !EqualsUtil.GeomExtensions.epsilonEquals(expectedTarget, m_drive.getTargetPose())) {
        Logger.recordOutput("PreAuto/TargetMismatch", true);
        m_led.updateState(LedState.kTargetMismatch);
      } else {
        Logger.recordOutput("PreAuto/TargetMismatch", false);
        m_led.updateState(LedState.kLocationCheck);
      }
    } else {
      m_led.updateState(LedState.kLocationCheck);
    }
  }

  public Pose2d getPathPlannerStartPose() {
    if (m_autoFactory != null && m_selectedAuto != null) {
      return m_autoFactory.getStartingPose(m_selectedAuto);
    }
    return new Pose2d();
  }

  public void setAutoSideLeft() {
    m_autoLeft = true;
  }

  public void setAutoSideRight() {
    m_autoLeft = false;
  }

  public void setBargeAuto(boolean newValue) {
    m_isBargeAuto = newValue;
  }

  public void setCitrusAuto(boolean newValue) {
    m_isCitrusAuto = newValue;
  }

  public void toggleTestingMode() {
    m_autoTestingMode = !m_autoTestingMode;
  }

  public void testAutoCoralIntake() {
    updateRobotAction(RobotAction.kAutoCoralIntaking);
  }

  public void testAutoAutoScore() {
    if (m_numCoralScoredAuto < 2) {
      if (m_autoLeft) {
        setReefIndexLeft();
      } else {
        setReefIndexRight();
      }
    } else if (m_numCoralScoredAuto < 3) {
      if (m_autoLeft) {
        setReefIndexRight();
      } else {
        setReefIndexLeft();
      }
    } else {
      // I. Hate. Math.
      Pose2d pose;
      if (m_isCitrusAuto) {
        if (m_autoLeft) {
          pose =
              new Pose2d(
                      AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                      Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? -120 : 60))
                  .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
        } else {
          pose =
              new Pose2d(
                      AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                      Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 120 : -60))
                  .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
        }
      } else {
        pose =
            new Pose2d(
                    AllianceFlipUtil.apply(FieldConstants.Reef.kCenter),
                    AllianceFlipUtil.apply(Rotation2d.fromDegrees(180)))
                .transformBy(new Transform2d(3.0, 0.0, new Rotation2d()));
      }
      if (m_autoLeft) {
        setReefIndexLeft(pose);
      } else {
        setReefIndexRight(pose);
      }
    }
    updateRobotAction(RobotAction.kAutoAutoScore);
  }

  public boolean getManipulatorAutoReversing() {
    return m_manipulator.getAutoReversing();
  }

  public DriveProfiles getDriveProfile() {
    return m_drive.getCurrentProfile();
  }

  public IntakeState getIntakeState() {
    return m_intake.getCurrentState();
  }

  public IndexerState getIndexerState() {
    return m_indexer.getCurrentState();
  }

  public ManipulatorState getManipulatorState() {
    return m_manipulator.getCurrentState();
  }

  public ClimbState getClimbState() {
    return m_climb.getCurrentState();
  }

  public ElevatorState getElevatorState() {
    return m_elevator.getCurrentState();
  }

  public LedState getLedState() {
    return m_led.getCurrentState();
  }
}
