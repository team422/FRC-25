package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class RobotState {

  // Subsystems
  private Drive m_drive;
  private AprilTagVision m_aprilTagVision;

  public enum RobotAction {
    kTeleopDefault,

    kAutoDefault,
  }

  private SubsystemProfiles<RobotAction> m_profiles;

  // Singleton logic
  private static RobotState m_instance;

  private RobotState(Drive drive, AprilTagVision aprilTagVision) {
    m_drive = drive;
    m_aprilTagVision = aprilTagVision;

    Map<RobotAction, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RobotAction.kTeleopDefault, () -> {});
    periodicHash.put(RobotAction.kAutoDefault, () -> {});

    m_profiles = new SubsystemProfiles<>(periodicHash, RobotAction.kTeleopDefault);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(Drive drive, AprilTagVision aprilTagVision) {
    if (m_instance == null) {
      m_instance = new RobotState(drive, aprilTagVision);
    }
    return m_instance;
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunction().run();
  }

  public void updateRobotAction(RobotAction newAction) {
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
    setDefaultAction();
  }

  public void onDisable() {}

  public void addVisionObservation(VisionObservation observation) {
    m_drive.addVisionObservation(observation);
  }

  public Pose2d getRobotPose() {
    return m_drive.getPose();
  }
}
