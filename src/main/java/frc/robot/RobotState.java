package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    kCoralIntaking, // run indexer only when the manipulator and elevator are in the right position
    kAlgaeIntaking, // run the intake until we have a gamepiece, then hold
    kAlgaeOuttaking,
    kAutoScore, // set the manipulator and elevator to the right position to score, use DriveToPoint
    // to get to the right position
    kClimbing,

    kAutoDefault,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

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
    periodicHash.put(RobotAction.kAlgaeIntaking, this::algaeIntakingPeriodic);
    periodicHash.put(RobotAction.kAlgaeOuttaking, () -> {});
    periodicHash.put(RobotAction.kCoralIntaking, this::coralIntakingPeriodic);
    periodicHash.put(RobotAction.kClimbing, this::climbingPeriodic);

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

  public void algaeIntakingPeriodic() {
    if (m_intake.hasGamePiece()) {
      m_intake.updateState(IntakeState.kGamepieceHold);
    }
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

  public void climbingPeriodic() {
    // TODO: implement later
  }

  public void updateRobotAction(RobotAction newAction) {
    switch (newAction) {
      case kAlgaeIntaking:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.updateState(IntakeState.kIntake);
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_led.setState(LedState.kEnabled);
        break;

      case kAlgaeOuttaking:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.updateState(IntakeState.kOuttake);
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_led.setState(LedState.kEnabled);

      case kAutoScore:
        // TODO: implement later with setpoint generator
        break;

      case kClimbing:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kDeploy);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.setState(LedState.kEnabled);
        break;

      case kCoralIntaking:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kIntaking);
        m_manipulator.updateState(ManipulatorState.kIntaking);
        m_led.setState(LedState.kEnabled);
        break;

      case kAutoDefault:
      case kTeleopDefault:
        m_drive.updateProfile(DriveProfiles.kDefault);
        m_intake.manageStowOrHold();
        m_indexer.updateState(IndexerState.kIdle);
        m_climb.updateState(ClimbState.kStow);
        m_elevator.updateState(ElevatorState.kStow);
        m_manipulator.updateState(ManipulatorState.kStow);
        m_led.setState(LedState.kEnabled);
        break;
    }
    m_profiles.setCurrentProfile(newAction);
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

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }

  public ChassisSpeeds getRobotSpeeds() {
    return m_drive.getChassisSpeeds();
  }
}
