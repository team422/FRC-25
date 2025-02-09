package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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

    kAutoDefault,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  private Timer m_timer = new Timer();

  private ReefHeight m_desiredReefHeight = ReefHeight.L1;
  private int m_desiredBranchIndex = 0;

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
    periodicHash.put(RobotAction.kCoralOuttaking, this::coralOuttakingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);
  }

  public static RobotState getInstance() {
    return m_instance;
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
    m_profiles.getPeriodicFunction().run();

    Logger.recordOutput("RobotState/CurrentAction", m_profiles.getCurrentProfile());
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

      case kCoralOuttaking:
        // don't change any other states since we want everything to stay in place
        m_manipulator.runRollerScoring();
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

  public void onEnable() {
    m_climb.zeroEncoder();
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
