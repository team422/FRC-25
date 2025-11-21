package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.util.SetpointGenerator;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  public enum RobotAction {
    kTeleopDefault,
    kScoring,
    kOuttaking,
    kIntaking,
    kAutoscore
  }

  private Drive m_drive;
  private Elevator m_elevator;
  private Manipulator m_manipulator;
  private Intake m_intake;
  private SubsystemProfiles<RobotAction> m_profiles;
  private static RobotState m_instance;
  private double m_desiredHeight = 0;
  private Pose2d m_desiredPose = new Pose2d();
  private boolean left = true;

  public RobotState(Drive drive, Elevator elevator, Manipulator manipulator, Intake intake) {
    m_drive = drive;
    m_elevator = elevator;
    m_manipulator = manipulator;
    m_intake = intake;

    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kTeleopDefault, () -> {});
    hash.put(RobotAction.kOuttaking, this::scoringPeriodic);
    hash.put(RobotAction.kScoring, this::scoringPeriodic);
    hash.put(RobotAction.kIntaking, this::intakingPeriodic);
    hash.put(RobotAction.kAutoscore, this::autoscorePeriodic);

    m_profiles = new SubsystemProfiles<>(hash, RobotAction.kTeleopDefault);
  }

  public void scoringPeriodic() {
    if (ready()) {
      if (m_elevator.atSetpoint(ElevatorConstants.kL1.getAsDouble())) {
        m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL1.getAsDouble()));
      } else if (m_elevator.atSetpoint(ElevatorConstants.kL2.getAsDouble())) {
        m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL2.getAsDouble()));
      } else if (m_elevator.atSetpoint(ElevatorConstants.kL3.getAsDouble())) {
        m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL3.getAsDouble()));
      } else if (m_elevator.atSetpoint(ElevatorConstants.kL4.getAsDouble())) {
        m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL4.getAsDouble()));
      } else {
        m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
      }
    } else {
      updateRobotAction(RobotAction.kTeleopDefault);
    }
  }

  public void intakingPeriodic() {
    if (ready()) {
      updateRobotAction(RobotAction.kTeleopDefault);
    }
  }

  public void autoscorePeriodic() {
    desiredPoses();

    // if (Math.abs(m_drive.getPose().getTranslation().getDistance(m_desiredPose.getTranslation()))
    //     < Units.inchesToMeters(DriveConstants.kAutoscoreDeployDistance.get())) {
    //       scoringPeriodic();
    //   if (m_elevator.getState() != ElevatorState.kScoring) {
    //     m_elevator.updateState(ElevatorState.kScoring);
    //   }
    //   if (m_elevator.atSetpoint(m_desiredHeight)
    //       && (m_manipulator.getState() != ManipulatorState.kScoring
    //           || m_manipulator.getState() != ManipulatorState.kOuttaking)) {
    //     m_manipulator.updateState(ManipulatorState.kScoring);
    //   }

    //   if(atSetpoints()) {
    //     m_manipulator.updateState(ManipulatorState.kOuttaking);
    //   }
    // }
  }

  public void desiredPoses() {
    if (!left) {
      if (!SetpointGenerator.getRightScore(m_drive.getPose()).equals(m_desiredPose)) {
        m_desiredPose = SetpointGenerator.getRightScore(m_drive.getPose());
        m_drive.setTargetPose(m_desiredPose);
      }
    } else {
      // TODO: add right
    }
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void updateRobotAction(RobotAction action) {
    DriveProfiles newDriveState = DriveProfiles.kDefault;
    ElevatorState newElevatorState = ElevatorState.kStow;
    ManipulatorState newManipState = ManipulatorState.kIdle;
    IntakeState newIntakeState = IntakeState.kIdle;

    switch (action) {
      case kAutoscore:
        newDriveState = DriveProfiles.kDriveToPoint;
        break;
      case kOuttaking:
        newManipState = ManipulatorState.kOuttaking;
        newElevatorState = ElevatorState.kScoring;
        break;
      case kScoring:
        newManipState = ManipulatorState.kScoring;
        newElevatorState = ElevatorState.kScoring;
        break;
      case kIntaking:
        newIntakeState = IntakeState.kIntaking;
        newManipState = ManipulatorState.kIntaking;
        break;
      default:
        break;
    }

    if (newElevatorState != m_elevator.getState()) {
      m_elevator.updateState(newElevatorState);
    }

    if (newManipState != m_manipulator.getState()) {
      m_manipulator.updateState(newManipState);
    }

    if (newIntakeState != m_intake.getState()) {
      m_intake.updateState(newIntakeState);
    }

    if (newDriveState != m_drive.getCurrentProfile()) {
      m_drive.updateProfile(newDriveState);
    }

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(
      Drive drive, Elevator elevator, Manipulator manipulator, Intake intake) {
    if (m_instance == null) {
      m_instance = new RobotState(drive, elevator, manipulator, intake);
    }
    return m_instance;
  }

  public RobotAction getCurrAction() {
    return m_profiles.getCurrentProfile();
  }

  public boolean atSetpoints() {
    return m_elevator.atSetpoint(m_desiredHeight) && m_manipulator.atSetpoint();
  }

  public void setHeight(double height) {
    m_desiredHeight = height;
    m_elevator.setHeight(height);
  }

  public boolean hasGamePiece() {
    return m_manipulator.hasGamePiece();
  }

  public boolean pieceInFunnel() {
    return m_manipulator.pieceInFunnel();
  }

  public boolean ready() {
    return m_manipulator.ready();
  }

  public void setLeft() {
    left = true;
    desiredPoses();
  }

  public void setRight() {
    left = false;
    desiredPoses();
  }
}
