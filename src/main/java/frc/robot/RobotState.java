package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  public enum RobotAction {
    kTeleopDefault,
    kScoring,
    kOuttaking
  }

  private Elevator m_elevator;
  private Manipulator m_manipulator;
  private SubsystemProfiles<RobotAction> m_profiles;
  private static RobotState m_instance;
  private double m_desiredHeight = 0;

  public RobotState(Elevator elevator, Manipulator manipulator) {
    m_elevator = elevator;
    m_manipulator = manipulator;

    HashMap<RobotAction, Runnable> m_hash = new HashMap<>();
    m_hash.put(RobotAction.kTeleopDefault, () -> {});
    m_hash.put(RobotAction.kOuttaking, this::scoringPeriodic);
    m_hash.put(RobotAction.kScoring, this::scoringPeriodic);

    m_profiles = new SubsystemProfiles<>(m_hash, RobotAction.kTeleopDefault);
  }

  public void scoringPeriodic() {
    if (m_elevator.atSetpoint(ElevatorConstants.kL1.getAsDouble())) {
      m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL1.getAsDouble()));
    } else if (m_elevator.atSetpoint(ElevatorConstants.kL2.getAsDouble())) {
      m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL2.getAsDouble()));
    } else if (m_elevator.atSetpoint(ElevatorConstants.kL3.getAsDouble())) {
      m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL3.getAsDouble()));
    } else if (m_elevator.atSetpoint(ElevatorConstants.kL4.getAsDouble())) {
      m_manipulator.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kL4.getAsDouble()));
    }
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void updateRobotAction(RobotAction action) {
    ElevatorState newElevatorState = ElevatorState.kStow;
    ManipulatorState newManipState = ManipulatorState.kIdle;

    switch (action) {
      case kOuttaking:
        newManipState = ManipulatorState.kScoring;
      case kScoring:
        // if (!pieceInFunnel() && hasGamePiece()) {
        newElevatorState = ElevatorState.kScoring;
        // }
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

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(Elevator elevator, Manipulator manipulator) {
    if (m_instance == null) {
      m_instance = new RobotState(elevator, manipulator);
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
}
