package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.littletonUtils.EqualsUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
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

@SuppressWarnings("unused")
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

  public enum RobotAction {
    kTeleopDefault,
    kCoralIntaking,
    kCoralOuttaking,
    kAlgaeIntakingOuttaking,
    kAutoScore,
    kManualScore,
    kClimbing,
    kDriveToProcessor,
    kBargeScore,
    kBargeOuttaking,

    // algae descore sequence
    kAlgaeDescoringInitial,
    kAlgaeDescoringDeployManipulator,
    kAlgaeDescoringMoveUp,
    kAlgaeDescoringFinal,

    kAutoDefault,
    kAutoAutoScore,
    kAutoCoralOuttaking,
  }

  private LedState currentLedState = LedState.kOff;

  private SubsystemProfiles<RobotAction> m_profiles;

  private Timer m_timer = new Timer();

  private ReefHeight m_desiredReefHeight = ReefHeight.L1;
  private int m_desiredBranchIndex = 0;

  private int m_numVisionGyroObservations = 0;

  private final List<RobotAction> kAlgaeDescoreSequence =
      List.of(
          RobotAction.kAlgaeDescoringInitial,
          RobotAction.kAlgaeDescoringDeployManipulator,
          RobotAction.kAlgaeDescoringMoveUp,
          RobotAction.kAlgaeDescoringFinal);

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
      AprilTagVision aprilTagVision) {
    m_drive = drive;
    m_intake = intake;
    m_indexer = indexer;
    m_manipulator = manipulator;
    m_climb = climb;
    m_elevator = elevator;
    m_led = led;
    m_aprilTagVision = aprilTagVision;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    periodicHash.put(RobotAction.kAlgaeIntakingOuttaking, this::algaeIntakingOuttakingPeriodic);
    periodicHash.put(RobotAction.kCoralIntaking, this::coralIntakingPeriodic);
    periodicHash.put(RobotAction.kClimbing, this::climbingPeriodic);
    periodicHash.put(RobotAction.kAutoScore, this::autoScorePeriodic);
    periodicHash.put(RobotAction.kManualScore, this::manualScorePeriodic);
    periodicHash.put(RobotAction.kDriveToProcessor, this::driveToProcessorPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringInitial, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kCoralOuttaking, this::coralOuttakingPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringInitial, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringDeployManipulator, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringMoveUp, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAlgaeDescoringFinal, this::algaeDescoringPeriodic);
    periodicHash.put(RobotAction.kAutoAutoScore, this::autoAutoScorePeriodic);
    periodicHash.put(RobotAction.kAutoCoralOuttaking, this::coralOuttakingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public void triggerAlert() {
    currentLedState = LedState.kAlert;
  }

  public static RobotState startInstance(
      Drive drive,
      Intake intake,
      Indexer indexer,
      Manipulator manipulator,
      Climb climb,
      Elevator elevator,
      Led led,
      AprilTagVision aprilTagVision) {
    if (m_instance == null) {
      m_instance =
          new RobotState(drive, intake, indexer, manipulator, climb, elevator, led, aprilTagVision);
    }
    return m_instance;
  }

  public void updateRobotState() {
    double start = HALUtil.getFPGATime();

    m_profiles.getPeriodicFunction().run();

    if (Constants.kUseComponents) {
      updateComponent();
    }

    Logger.recordOutput("RobotState/CurrentAction", m_profiles.getCurrentProfile());

    Logger.recordOutput("PeriodicTime/RobotState", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void coralIntakingPeriodic() {
    // when we have a game piece don't runm
    if (m_manipulator.hasGamePiece()) {
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
    if (m_timer.hasElapsed(0.5)) {
      // after we release the coral go back to stow
      setDefaultAction();
    }
    if (!m_timer.isRunning() && !m_manipulator.hasGamePiece()) {
      m_timer.restart();
    }
  }

  public void algaeIntakingOuttakingPeriodic() {
    // we could be intaking or outtaking here
    // if we're outtaking we don't want to go to hold
    if (m_intake.getCurrentState() == IntakeState.kIntake) {
      if (m_intake.hasGamePiece()) {
        m_intake.updateState(IntakeState.kGamepieceHold);
      }
    }
  }

  public void autoScorePeriodic() {
    RobotSetpoint setpoint = SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight);
    // only set the setpoints if they're different so we don't reset the motion profile or drive pid
    if (!EqualsUtil.GeomExtensions.epsilonEquals(setpoint.drivePose(), m_drive.getTargetPose())) {
      m_drive.setTargetPose(setpoint.drivePose());
    }

    // we don't want to deploy elevator or manipulator until we're close to the setpoint
    if (Math.abs(
            m_drive
                .getTargetPose()
                .getTranslation()
                .getDistance(setpoint.drivePose().getTranslation()))
        < Units.inchesToMeters(6)) {
      if (!EqualsUtil.epsilonEquals(setpoint.elevatorHeight(), m_elevator.getDesiredHeight())) {
        m_elevator.setDesiredHeight(setpoint.elevatorHeight());
      }
      if (!EqualsUtil.epsilonEquals(
          setpoint.manipulatorAngle().getRadians(),
          m_manipulator.getDesiredWristAngle().getRadians())) {
        m_manipulator.setDesiredWristAngle(setpoint.manipulatorAngle());
      }
    }
  }

  public void autoAutoScorePeriodic() {
    autoScorePeriodic();

    // everything must be within tolerance to run the rollers
    if (m_elevator.atSetpoint()
        && m_manipulator.atSetpoint()
        // when the command finishes, drive will go back to default
        && m_drive.getCurrentProfile() == DriveProfiles.kDefault) {
      updateRobotAction(RobotAction.kAutoCoralOuttaking);
    }
  }

  public void manualScorePeriodic() {
    RobotSetpoint setpoint = SetpointGenerator.generate(m_desiredBranchIndex, m_desiredReefHeight);
    // only set the setpoints if they're different so we don't reset the motion profile
    if (!EqualsUtil.epsilonEquals(setpoint.elevatorHeight(), m_elevator.getDesiredHeight())) {
      m_elevator.setDesiredHeight(setpoint.elevatorHeight());
    }
    if (!EqualsUtil.epsilonEquals(
        setpoint.manipulatorAngle().getRadians(),
        m_manipulator.getDesiredWristAngle().getRadians())) {
      m_manipulator.setDesiredWristAngle(setpoint.manipulatorAngle());
    }
    // don't worry about drive in manual score
  }

  public void driveToProcessorPeriodic() {
    Pose2d processorPose = FieldConstants.Processor.kCenterFace;
    if (!EqualsUtil.GeomExtensions.epsilonEquals(processorPose, m_drive.getTargetPose())) {
      m_drive.setTargetPose(processorPose);
    }
  }

  public void climbingPeriodic() {
    // TODO: come back here, i don't think this needs anything but it feels like it should
  }

  public void bargeScorePeriodic() {
    // TODO: when we trust the vision more we will add full auto score (rollers decide when to go)
  }

  public void algaeDescoringPeriodic() {
    // each step in the sequence is identical in terms of whether to advance to the next step
    int index = kAlgaeDescoreSequence.indexOf(m_profiles.getCurrentProfile());
    if (m_elevator.atSetpoint()
        && m_manipulator.atSetpoint()
        && index < kAlgaeDescoreSequence.size() - 1) {
      m_profiles.setCurrentProfile(kAlgaeDescoreSequence.get(index + 1));
    }
  }

  public void updateRobotAction(RobotAction newAction) {
    switch (newAction) {
      case kAlgaeIntakingOuttaking:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageIntakeOrOuttake();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.updateState(LedState.kEnabled);
        break;

      case kManualScore:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kScoring);
        m_manipulator.updateState(ManipulatorState.kScoring);
        m_led.updateState(LedState.kAutoScore);
        // guys don't use fallthrough it's not worth it
        break;

      case kAutoScore:
      case kAutoAutoScore:
        m_drive.updateProfile(DriveProfiles.kDriveToPoint);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kScoring);
        m_manipulator.updateState(ManipulatorState.kScoring);
        m_led.updateState(LedState.kAutoScore);
        break;

      case kDriveToProcessor:
        m_drive.updateProfile(DriveProfiles.kDriveToPoint);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.updateState(LedState.kEnabled);
        break;

      case kClimbing:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kDeploy);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.updateState(LedState.kEnabled);
        break;

      case kCoralIntaking:
        // TODO: intake needs to move out a little so it doesn't get in way of funnel
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kIntaking);
        m_manipulator.updateState(ManipulatorState.kIntaking);
        m_led.updateState(LedState.kEnabled);
        break;

      case kAutoDefault:
      case kTeleopDefault:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.updateState(LedState.kEnabled);
        break;

      case kAutoCoralOuttaking:
      case kCoralOuttaking:
        // don't change any other states since we want everything to stay in place
        m_manipulator.runRollerScoring();
        break;

      case kBargeOuttaking:
        // don't change any other states since we want everything to stay in place
        // name is kinda confusing but we would still be in the algae descore sequence on the
        // manipulator
        m_manipulator.runRollerAlgaeDescoring();
        break;

      case kBargeScore:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.updateState(LedState.kEnabled);
        break;

      case kAlgaeDescoringInitial:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kAlgaeDescoringInitial);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.updateState(LedState.kEnabled);
        break;

      case kAlgaeDescoringDeployManipulator:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kAlgaeDescoringInitial);
        m_manipulator.updateState(ManipulatorState.kAlgaeDescoring);
        m_led.updateState(LedState.kEnabled);
        break;

      case kAlgaeDescoringMoveUp:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kAlgaeDescoringFinal);
        m_manipulator.updateState(ManipulatorState.kAlgaeDescoring);
        m_led.updateState(LedState.kEnabled);
        break;

      case kAlgaeDescoringFinal:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kAlgaeDescoring);
        m_manipulator.stopRollerAlgaeDescoring();
        m_led.updateState(LedState.kEnabled);
        break;
    }
    m_profiles.setCurrentProfile(newAction);

    // stop the timer so a previous action doesn't affect the new one
    m_timer.stop();
    m_timer.reset();
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
    if (SetpointGenerator.isNearProcessor(m_drive.getPose())) {
      updateRobotAction(RobotAction.kDriveToProcessor);
    } else if (SetpointGenerator.isNearBarge(m_drive.getPose())) {
      updateRobotAction(RobotAction.kBargeScore);
    } else {
      updateRobotAction(RobotAction.kAutoScore);
    }
  }

  public void manageCoralOuttakePressed() {
    // this button will score coral but also score algae if we're at the barge
    if (m_profiles.getCurrentProfile() == RobotAction.kBargeScore) {
      updateRobotAction(RobotAction.kBargeOuttaking);
    } else {
      updateRobotAction(RobotAction.kCoralOuttaking);
    }
  }

  public void onEnable() {
    setDefaultAction();
  }

  public void onDisable() {}

  public void addVisionObservation(VisionObservation observation) {
    m_drive.addVisionObservation(observation);
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
    var currPose = getRobotPose();
    var indices = SetpointGenerator.getPossibleIndices(currPose);
    // indices are in order starting from the driver station and going clockwise
    // so the second index is the one to the left
    if (indices.getSecond() != -1) {
      m_desiredBranchIndex = indices.getSecond();
    }
  }

  public void setReefIndexRight() {
    var currPose = getRobotPose();
    var indices = SetpointGenerator.getPossibleIndices(currPose);
    // indices are in order starting from the driver station and going clockwise
    // so the first index is the one to the right
    if (indices.getFirst() != -1) {
      m_desiredBranchIndex = indices.getFirst();
    }
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
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
                Degrees.of(-150 + m_intake.getRotation().getDegrees()),
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
                Meters.of(0.285), Meters.zero(), Meters.of(0.203 + m_elevator.getCurrHeight())),
            // new Translation3d(-0.31, 0, 0.19),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(-90 + m_manipulator.getCurrAngle().getDegrees()),
                Degrees.of(0)));
    Pose3d climbPose =
        new Pose3d( // static for now
            new Translation3d(0, -0.336, 0.405),
            new Rotation3d(Degrees.of(-90 + 0), Degrees.of(0), Degrees.of(0)));
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

  public int getNumVisionGyroObservations() {
    return m_numVisionGyroObservations;
  }

  public void incrementNumVisionGyroObservations() {
    m_numVisionGyroObservations++;
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
