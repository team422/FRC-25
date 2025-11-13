package frc.robot;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  public enum RobotAction {
    kTeleopDefault,
    kScoring
  }

  private Elevator m_elevator;
  private SubsystemProfiles<RobotAction> m_profiles;
  private static RobotState m_instance;

  public RobotState(Elevator elevator) {
    m_elevator = elevator;

    HashMap<RobotAction, Runnable> m_hash = new HashMap<>();
    m_hash.put(RobotAction.kTeleopDefault, () -> {});
    m_hash.put(RobotAction.kScoring, this::scoringPeriodic);

    m_profiles = new SubsystemProfiles<>(m_hash, RobotAction.kTeleopDefault);
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void scoringPeriodic() {
    m_elevator.updateState(ElevatorState.kScoring);
  }

  public void updateRobotAction(RobotAction action) {
    ElevatorState newElevatorState = ElevatorState.kStow;

    switch (action) {
      case kScoring:
        newElevatorState = ElevatorState.kScoring;
        break;

      default:
        break;
    }

    if (newElevatorState != m_elevator.getState()) {
      m_elevator.updateState(newElevatorState);
    }

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(Elevator elevator) {
    if (m_instance == null) {
      m_instance = new RobotState(elevator);
    }
    return m_instance;
  }

  public RobotAction getCurrAction() {
    return m_profiles.getCurrentProfile();
  }
}
