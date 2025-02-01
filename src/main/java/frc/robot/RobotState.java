package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVision.VisionObservation;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.manipulator.Manipulator;
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

    if (Constants.kUseComponents) {
      updateComponent();
    }
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

  public void updateComponent() {
    Pose3d intakePose =
        new Pose3d( // Algae Intake
            new Translation3d(-0.31, 0, 0.19),
            new Rotation3d(
                Units.Degrees.of(0),
                Units.Degrees.of(-150 + m_intake.getRotation().getDegrees()),
                Units.Degrees.of(180)));
    Pose3d elevatorStage2Pose =
        new Pose3d(
            new Translation3d(
                Units.Meters.zero(),
                Units.Meters.zero(),
                Units.Meters.of(Math.max(0, m_elevator.getCurrHeight() - 45))),
            new Rotation3d());
    Pose3d elevatorStage3Pose =
        new Pose3d(
            new Translation3d(
                Units.Meters.zero(),
                Units.Meters.zero(),
                Units.Meters.of(Math.max(0, m_elevator.getCurrHeight() - 18))),
            new Rotation3d());
    Pose3d carriagePose =
        new Pose3d(
            new Translation3d(
                Units.Meters.zero(),
                Units.Meters.zero(),
                Units.Meters.of(m_elevator.getCurrHeight())),
            new Rotation3d());
    Pose3d manipulatorPose =
        new Pose3d( // Manipulator
            new Translation3d(
                0.285, 0, 0.203 + Units.Meters.of(m_elevator.getCurrHeight()).in(Units.Meters)),
            // new Translation3d(-0.31, 0, 0.19),
            new Rotation3d(
                Units.Degrees.of(0),
                Units.Degrees.of(-90 + m_manipulator.getPose().getDegrees()),
                Units.Degrees.of(0)));
    Pose3d climbPose =
        new Pose3d( // static for now
            new Translation3d(0, -0.336, 0.405),
            new Rotation3d(Units.Degrees.of(-90 + 0), Units.Degrees.of(0), Units.Degrees.of(0)));
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
}
