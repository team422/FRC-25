package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.SwerveSetpointGenerator.ModuleLimits;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final boolean kTuningMode = true;

  public static final Mode kRealMode = Mode.PROTO;
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

  // set to false to disable the base refresh manager
  public static final boolean kUseBaseRefreshManager = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running on the proto board */
    PROTO,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final double kMaxLinearSpeed = 6.0; // meters per second
    public static final double kMaxLinearAcceleration = 3.0; // meters per second squared
    public static final double kTrackWidthX = Units.inchesToMeters(28.0);
    public static final double kTrackWidthY = Units.inchesToMeters(28.0);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;
    public static final double kMaxAngularAcceleration = kMaxLinearAcceleration / kDriveBaseRadius;
    public static final LoggedTunableNumber kTeleopRotationSpeed =
        new LoggedTunableNumber("Teleop Rotation Speed", 10.0);

    public static final Translation2d[] kModuleTranslations =
        new Translation2d[] {
          new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
        };

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final ModuleLimits kModuleLimitsFree =
        new ModuleLimits(kMaxLinearSpeed, kMaxAngularSpeed, Units.degreesToRadians(1080.0));

    public static final double kWheelRadius = Units.inchesToMeters(2.0);
    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveGearRatio =
        (50.0 / 16.0) * (19.0 / 25.0) * (45.0 / 15.0); // L1+ gear ratio
    public static final double kTurnGearRatio = 150.0 / 7.0;

    // Simulation constants
    public static final double kDriveSimGearRatio = kDriveGearRatio;
    public static final double kDriveSimMOI = 0.025;
    public static final double kTurnSimGearRatio = kTurnGearRatio;
    public static final double kTurnSimMOI = 0.004;

    public static final LoggedTunableNumber kHeadingP =
        new LoggedTunableNumber("Drive Heading P", 4.0);
    public static final LoggedTunableNumber kHeadingI =
        new LoggedTunableNumber("Drive Heading I", 0.0);
    public static final LoggedTunableNumber kHeadingD =
        new LoggedTunableNumber("Drive Heading D", 0.05);

    // universal reversals for drive (aka the big negative sign)
    public static final boolean kRealReversed = false;
    public static final boolean kSimReversed = false;

    public static final LoggedTunableNumber kDriveToPointP =
        new LoggedTunableNumber("DriveToPoint P", 3.0);
    public static final LoggedTunableNumber kDriveToPointI =
        new LoggedTunableNumber("DriveToPoint I", 0.0);
    public static final LoggedTunableNumber kDriveToPointD =
        new LoggedTunableNumber("DriveToPoint D", 0.0);

    public static final LoggedTunableNumber kDriveToPointHeadingP =
        new LoggedTunableNumber("DriveToPoint Heading P", 3.0);
    public static final LoggedTunableNumber kDriveToPointHeadingI =
        new LoggedTunableNumber("DriveToPoint Heading I", 0.0);
    public static final LoggedTunableNumber kDriveToPointHeadingD =
        new LoggedTunableNumber("DriveToPoint Heading D", 0.0);
  }

  public static final class ClimbConstants {

    public static final LoggedTunableNumber kClimbP = new LoggedTunableNumber("Climb P", 3);
    public static final LoggedTunableNumber kClimbI = new LoggedTunableNumber("Climb I", 0.05);
    public static final LoggedTunableNumber kClimbD = new LoggedTunableNumber("Climb D", 0.4);
    public static final double kClimbTolerance = 0.5; // degrees

    public static final LoggedTunableNumber kStowFeedforward =
        new LoggedTunableNumber("Climb Stow Feedforward", 0.0);

    public static final LoggedTunableNumber kClimbStowPos =
        new LoggedTunableNumber("Climb Stow Rad", 15.0); // degrees
    public static final LoggedTunableNumber kClimbDeployPos =
        new LoggedTunableNumber("Climb Deploy Rad", 240); // degrees

    public static final double kClimbReduction = (5 / 1) * (4 / 1) * (68 / 18);

    // sim constants
    public static final double kSimGearing = 1.0;
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kSimClimbArmLengthMeters = 0.25;
    public static final double kSimMOI =
        SingleJointedArmSim.estimateMOI(kSimClimbArmLengthMeters, 5.5);
    public static final double kSimMinAngleRad =
        Units.degreesToRadians(
            -30); // vertical is 0deg, pos deg is towards outside of robot in position to grab cage
    public static final double kSimMaxAngleRad = Units.degreesToRadians(360);
    public static final double kSimStartingAngleRad = Units.degreesToRadians(0);
    public static final boolean kSimGravity = false;
  }

  public static final class LedConstants {
    public static final int kStripLength = 60;

    public static final Color kOff = Color.kRed;
    public static final Color kAutoscore = Color.kGreen;
    public static final Color kHasGampiece = Color.kDarkMagenta;
    public static final Color kEnabled = Color.kYellow;
    public static final Color kDisabled = Color.kRed;
    public static final Color kAlert = Color.kGold;
  }

  public static final class ElevatorConstants {
    public static final double kTopSpeed = 1;
    public static final double kTopAcceleration = 5;
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator P", 1.0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator I", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator D", 0.0);
    public static final LoggedTunableNumber kKS = new LoggedTunableNumber("Elevator kS", 0.0);
    public static final LoggedTunableNumber kKG = new LoggedTunableNumber("Elevator kG", 0.0);
    public static final LoggedTunableNumber kKV = new LoggedTunableNumber("Elevator kV", 0.2);
    public static final LoggedTunableNumber kKA = new LoggedTunableNumber("Elevator kA", 0.0);

    public static final LoggedTunableNumber kMagicMotionCruiseVelocity =
        new LoggedTunableNumber("Elevator MagicMotion cruise velocity", 5.0);
    public static final LoggedTunableNumber kMagicMotionAcceleration =
        new LoggedTunableNumber("Elevator MagicMotion acceleration", 10.0);
    public static final LoggedTunableNumber kMagicMotionJerk =
        new LoggedTunableNumber("Elevator MagicMotion Jerk", 100.0);

    public static final double kStowHeight = 0;
    public static final double kIntakingHeight = 0.5;
    public static final double kKnockingHeight = 2;

    public static final double kRadius = Units.inchesToMeters(2.256);
    public static final double kGearRatio = 54 / 12;
    public static final double kSensorToMechanismRatio = 2 * kRadius * Math.PI * kGearRatio;
    public static final LoggedTunableNumber kElevatorOffset =
        new LoggedTunableNumber("Elevator/Offset", Units.inchesToMeters(0));

    // Simulation constants
    public static final double kSimGearing = kGearRatio;
    public static final double kSimMOI = .001;
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(2);
    public static final double kMinHeight = 0;
    public static final double kMaxHeight = 2;
    public static final double kHeightTolerance = .10;
  }

  public static final class AprilTagVisionConstants {
    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);

    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    // z margin is much smaller because the camera should be perfectly level with the tag
    public static final double kZMargin = Units.inchesToMeters(1);

    public static final double kErrorStandardDeviationThreshold = 0.2; // acceptable error

    public static final LoggedTunableNumber kXYStandardDeviationCoefficient =
        new LoggedTunableNumber("xyStandardDeviationCoefficient", 0.005);
    public static final LoggedTunableNumber kThetaStandardDeviationCoefficient =
        new LoggedTunableNumber("thetaStandardDeviationCoefficient", 0.01);

    // transform from center of robot to camera
    public static final Transform3d[] kCameraTransforms =
        new Transform3d[] {
          // front right (shooter)
          new Transform3d(
              Units.inchesToMeters(9.454),
              Units.inchesToMeters(-5.541),
              Units.inchesToMeters(7.766),
              new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-9.97)))),

          // back right (intake)
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-14.620),
                  Units.inchesToMeters(4.673),
                  Units.inchesToMeters(8.585)),
              new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(180 - 10))),

          // back left (intake)
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-14.620),
                  Units.inchesToMeters(4.673),
                  Units.inchesToMeters(8.585)),
              new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(180 - 10))),

          // front left (shooter)
          new Transform3d(
              Units.inchesToMeters(9.454),
              Units.inchesToMeters(5.541),
              Units.inchesToMeters(7.766),
              new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(9.97)))),
        };

    // tolerances for using the vision rotation, temp values
    public static final double kRotationErrorThreshold = 0.1;
    public static final double kRotationDistanceThreshold = Units.inchesToMeters(8);
    public static final double kRotationSpeedThreshold = 0.2; // m/s

    public static final int kCalibIndex = 3;

    public static final LoggedTunableNumber transformCameraX =
        new LoggedTunableNumber("cameraX", kCameraTransforms[kCalibIndex].getX());
    public static final LoggedTunableNumber cameraY =
        new LoggedTunableNumber("cameraY", kCameraTransforms[kCalibIndex].getY());
    public static final LoggedTunableNumber cameraZ =
        new LoggedTunableNumber("cameraZ", kCameraTransforms[kCalibIndex].getZ());
    public static final LoggedTunableNumber cameraRoll =
        new LoggedTunableNumber("cameraRoll", kCameraTransforms[kCalibIndex].getRotation().getX());
    public static final LoggedTunableNumber cameraPitch =
        new LoggedTunableNumber("cameraPitch", kCameraTransforms[kCalibIndex].getRotation().getY());
    public static final LoggedTunableNumber cameraYaw =
        new LoggedTunableNumber("cameraYaw", kCameraTransforms[kCalibIndex].getRotation().getZ());
  }

  public static final class IntakeConstants {
    // public static final double kPivotGearRatio = (84.0 / 8.0) * (36.0 / 14.0);
    public static final double kPivotGearRatio = 1.0;
    public static final double kRollerGearRatio = (30.0 / 12.0);
    public static final double kRollerRadius = Units.inchesToMeters(1.5);

    public static final double kPivotTolerance = 2.0; // degrees

    public static final Rotation2d kPivotOffset = Rotation2d.fromDegrees(0.0);

    public static final double kRollerCurrentGamepieceThreshold =
        0.5; // amps to be considered holding a gamepiece, temp value
    public static final double kRollerAccelGamepieceThreshold =
        1.0; // rotations per second squared to be considered holding a gamepiece, temp value

    public static final LoggedTunableNumber kPivotP = new LoggedTunableNumber("Pivot P", 0.0);
    public static final LoggedTunableNumber kPivotI = new LoggedTunableNumber("Pivot I", 0.0);
    public static final LoggedTunableNumber kPivotD = new LoggedTunableNumber("Pivot D", 0.0);
    public static final LoggedTunableNumber kPivotKS = new LoggedTunableNumber("Pivot kS", 0.0);
    public static final LoggedTunableNumber kPivotKG = new LoggedTunableNumber("Pivot kG", 0.0);

    public static final LoggedTunableNumber kPivotStowAngle =
        new LoggedTunableNumber("Pivot Stow Angle", 0.0);
    public static final LoggedTunableNumber kPivotIntakeAngle =
        new LoggedTunableNumber("Pivot Intake Angle", 150.0);
    public static final LoggedTunableNumber kPivotHoldAngle =
        new LoggedTunableNumber("Pivot Hold Angle", 75.0);
    public static final LoggedTunableNumber kPivotOuttakeAngle =
        new LoggedTunableNumber("Pivot Outtake Angle", 150.0);

    public static final LoggedTunableNumber kRollerStowVoltage =
        new LoggedTunableNumber("Intake Roller Stow Voltage", 0.0);
    public static final LoggedTunableNumber kRollerIntakeVoltage =
        new LoggedTunableNumber("Intake Roller Intake Voltage", 2.0);
    public static final LoggedTunableNumber kRollerHoldVoltage =
        new LoggedTunableNumber("Intake Roller Hold Voltage", 0.5);
    public static final LoggedTunableNumber kRollerOuttakeVoltage =
        new LoggedTunableNumber("Intake Roller Outtake Voltage", -2.0);

    // Simulation constants
    public static final DCMotor kPivotSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kPivotSimGearing = kPivotGearRatio;
    public static final double kPivotArmMass = Units.lbsToKilograms(3.419);
    public static final double kPivotArmLength = Units.inchesToMeters(17.5);
    public static final double kPivotSimMOI =
        SingleJointedArmSim.estimateMOI(kPivotArmLength, kPivotArmMass);
    public static final Rotation2d kPivotMinAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kPivotMaxAngle = Rotation2d.fromDegrees(65);
    public static final boolean kSimSimulateGravity = true;
    public static final Rotation2d kSimStartingAngle = kPivotMinAngle;

    public static final DCMotor kRollerSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kRollerSimGearing = kRollerGearRatio;
    public static final double kRollerSimMOI = 0.004;
  }

  public static final class ManipulatorConstants {
    public static final double kWristGearRatio = (66.0 / 10.0) * (32.0 / 14.0);
    public static final double kRollerGearRatio = (58.0 / 16.0);
    public static final double kRollerRadius = Units.inchesToMeters(3);

    public static final double kWristTolerance = 2.0; // degrees

    public static final Rotation2d kWristOffset = Rotation2d.fromDegrees(0.0);

    public static final LoggedTunableNumber kWristP = new LoggedTunableNumber("Wrist P", 1.0);
    public static final LoggedTunableNumber kWristI = new LoggedTunableNumber("Wrist I", 0.0);
    public static final LoggedTunableNumber kWristD = new LoggedTunableNumber("Wrist D", 0.0);
    public static final LoggedTunableNumber kWristKS = new LoggedTunableNumber("Wrist kS", 0.0);
    public static final LoggedTunableNumber kWristKG = new LoggedTunableNumber("Wrist kG", 0.0);

    public static final LoggedTunableNumber kWristStowAngle =
        new LoggedTunableNumber("Wrist Stow Angle", 0.0);
    public static final LoggedTunableNumber kWristIntakeAngle =
        new LoggedTunableNumber("Wrist Intake Angle", 50.0);
    public static final LoggedTunableNumber kWristScoringOffset =
        new LoggedTunableNumber("Wrist Scoring Offset", 0.0);

    public static final LoggedTunableNumber kRollerStowVoltage =
        new LoggedTunableNumber("Manipulator Roller Stow Voltage", 0.0);
    public static final LoggedTunableNumber kRollerIntakeVoltage =
        new LoggedTunableNumber("Manipulator Roller Intake Voltage", 2.0);
    public static final LoggedTunableNumber kRollerScoringVoltage =
        new LoggedTunableNumber("Manipulator Roller Scoring Voltage", 0.0);

    // Simulation constants
    public static final DCMotor kWristSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kWristSimGearing = kWristGearRatio;
    public static final double kWristArmMass = Units.lbsToKilograms(3.373);
    public static final double kWristArmLength = Units.inchesToMeters(7.75);
    public static final double kWristSimMOI =
        SingleJointedArmSim.estimateMOI(kWristArmLength, kWristArmMass);
    public static final Rotation2d kWristMinAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kWristMaxAngle = Rotation2d.fromDegrees(130.0);
    public static final boolean kSimSimulateGravity = true;
    public static final Rotation2d kSimStartingAngle = kWristMinAngle;

    public static final DCMotor kRollerSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kRollerSimGearing = kRollerGearRatio;
    public static final double kRollerSimMOI = 0.004;
  }

  public static final class CurrentLimitConstants {
    // Drive
    public static final double kDriveDefaultSupplyCurrentLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 75.0;
    public static final double kDriveDefaultStatorCurrentLimit = 180.0;

    public static final double kTurnDefaultSupplyCurrentLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 30.0;
    public static final double kTurnDefaultStatorCurrentLimit = 120.0;

    // Intake
    public static final double kIntakePivotDefaultSupplyLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 80.0;
    public static final double kIntakePivotDefaultStatorLimit = 120.0;

    public static final double kIntakeRollerDefaultSupplyLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 80.0;
    public static final double kIntakeRollerDefaultStatorLimit = 120.0;

    // Indexer
    public static final double kIndexerDefaultSupplyLimit = kCurrentMode == Mode.PROTO ? 5.0 : 30.0;
    public static final double kIndexerDefaultStatorLimit = 120.0;

    // Manipulator
    public static final double kManipulatorWristDefaultSupplyLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 80.0;
    public static final double kManipulatorWristDefaultStatorLimit = 120.0;

    public static final double kManipulatorRollerDefaultSupplyLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 80.0;
    public static final double kManipulatorRollerDefaultStatorLimit = 120.0;

    // Elevator
    public static final double kElevatorDefaultSupplyLimit =
        kCurrentMode == Mode.PROTO ? 5.0 : 65.0;
    public static final double kElevatorDefaultStatorLimit = 95.0;

    // Climb
    public static final double kClimbDefaultSupplyLimit = kCurrentMode == Mode.PROTO ? 5.0 : 80.0;
    public static final double kClimbDefaultStatorLimit = 120.0;
  }

  public static final class IndexerConstants {

    public static final double kGearRatio = 30 / 8;
    public static final double kRollerRadius = Units.inchesToMeters(2);

    public static final LoggedTunableNumber kIndexerIdleVoltage =
        new LoggedTunableNumber("Indexer Idle Voltage", 0.0);
    public static final LoggedTunableNumber kIndexerIndexingVoltage =
        new LoggedTunableNumber("Indexer Indexing Voltage", 2.0);

    // Simulation constants
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kSimGearing = kGearRatio;
    public static final double kSimMOI = 0.005;
  }

  public static final class Ports {
    public static final int kFrontLeftDrive = 0;
    public static final int kFrontLeftTurn = 1;
    public static final int kFrontLeftCancoder = 2;

    public static final int kFrontRightDrive = 3;
    public static final int kFrontRightTurn = 4;
    public static final int kFrontRightCancoder = 5;

    public static final int kBackLeftDrive = 6;
    public static final int kBackLeftTurn = 7;
    public static final int kBackLeftCancoder = 8;

    public static final int kBackRightDrive = 9;
    public static final int kBackRightTurn = 10;
    public static final int kBackRightCancoder = 11;

    public static final int kPigeon = 22;

    public static final String kDriveCanivoreName = "Drivetrain";

    public static final int kClimbMotor = 12;
    public static final int kIntakeRoller = 0;
    public static final int kIntakePivot = 28;

    public static final int kManipulatorRoller = 28;
    public static final int kManipulatorWrist = 0;

    public static final int kIntakeAbsoluteEncoder = 9;

    public static final int kIndexerMotor = 31;

    public static final int kManipulatorAbsoluteEncoder = 9;

    public static final int kPhotoElectricOne = 0;
    public static final int kPhotoElectricTwo = 3;

    public static final int kLed = 2;

    public static final int kElevatorLead = 7;
    public static final int kElevatorFollowing = 9;

    public static final String kMainCanivoreName = "Drivetrain";
  }

  /** Whether or not subsystems are enabled on the proto board */
  public static final class ProtoConstants {
    public static final boolean kRealDrive = false;
    public static final boolean kRealIntake = false;
    public static final boolean kRealIndexer = false;
    public static final boolean kRealManipulator = false;
    public static final boolean kRealClimb = false;
    public static final boolean kRealElevator = true;
  }

  public class FieldConstants {

    // FieldConstants taken from 6328 Mechanical Advantage
    // Please don't sue me jwbonner
    // I love your code
    public static final double kFieldLength = Units.inchesToMeters(690.876);
    public static final double kFieldWidth = Units.inchesToMeters(317);
    public static final double kStartingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public static class Processor {
      public static final Pose2d kCenterFace =
          new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
      public static final Translation2d kFarCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
      public static final Translation2d kMiddleCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
      public static final Translation2d kCloseCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

      // Measured from floor to bottom of cage
      public static final double kDeepHeight = Units.inchesToMeters(3.125);
      public static final double kShallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
      public static final Pose2d kLeftCenterFace =
          new Pose2d(
              Units.inchesToMeters(33.526),
              Units.inchesToMeters(291.176),
              Rotation2d.fromDegrees(90 - 144.011));
      public static final Pose2d kRightCenterFace =
          new Pose2d(
              Units.inchesToMeters(33.526),
              Units.inchesToMeters(25.824),
              Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
      public static final Translation2d kCenter =
          new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
      public static final double kFaceToZoneLine =
          Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

      public static final Pose2d[] kCenterFaces =
          new Pose2d[6]; // Starting facing the driver station in clockwise order
      public static final List<Map<ReefHeight, Pose3d>> kBranchPositions =
          new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise

      static {
        // Initialize faces
        kCenterFaces[0] =
            new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(180));
        kCenterFaces[1] =
            new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(120));
        kCenterFaces[2] =
            new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(60));
        kCenterFaces[3] =
            new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(0));
        kCenterFaces[4] =
            new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(-60));
        kCenterFaces[5] =
            new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(-120));

        // Initialize branch positions
        for (int face = 0; face < 6; face++) {
          Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
          Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
          for (var level : ReefHeight.values()) {
            Pose2d poseDirection = new Pose2d(kCenter, Rotation2d.fromDegrees(180 - (60 * face)));
            double adjustX = Units.inchesToMeters(30.738);
            double adjustY = Units.inchesToMeters(6.469);

            fillRight.put(
                level,
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians())));
            fillLeft.put(
                level,
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians())));
          }
          kBranchPositions.add(fillRight);
          kBranchPositions.add(fillLeft);
        }
      }
    }

    public static class StagingPositions {
      // Measured from the center of the ice cream
      public static final Pose2d kLeftIceCream =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
      public static final Pose2d kMiddleIceCream =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
      public static final Pose2d kRightIceCream =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
      L4(Units.inchesToMeters(72), -90),
      L3(Units.inchesToMeters(47.625), -35),
      L2(Units.inchesToMeters(31.875), -35),
      L1(Units.inchesToMeters(18), 0);

      ReefHeight(double height, double pitch) {
        this.height = height;
        this.pitch = pitch; // in degrees
      }

      public final double height;
      public final double pitch;
    }
  }
}
