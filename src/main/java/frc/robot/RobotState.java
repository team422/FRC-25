package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.roller.Roller;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class RobotState {

  // Subsystems
  private Drive m_drive;
  private Roller m_roller;

  public enum RobotAction {
    kTeleopDefault,
    kAutoDefault,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  // Singleton logic
  private static RobotState m_instance;

  private RobotState(Drive drive, Roller roller) {
    m_drive = drive;
    m_roller = roller;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kAutoDefault, () -> {});
    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(Drive drive, Roller roller) {
    if (m_instance == null) {
      m_instance = new RobotState(drive, roller);
    }
    return m_instance;
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunction().run();
  }

  public void updateRobotAction(RobotAction newAction) {
    m_profiles.setCurrentProfile(newAction);
  }

  public void setDefaultAction() {
    if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
      updateRobotAction(RobotAction.kAutoDefault);
    } else {
      updateRobotAction(RobotAction.kTeleopDefault);
    }
  }

  public void onEnable() {}

  public void onDisable() {}
}
