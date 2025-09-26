package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.littletonUtils.GeomUtil;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.lib.littletonUtils.SwerveSetpointGenerator.ModuleLimits;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.RequiredArgsConstructor;

public final class Constants {
  public static final boolean kTuningMode = true;

  public static final Mode kRealMode = Mode.REAL;
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

  public static final boolean kUsePhoenixDiagnosticServer = false;

  // set to false to disable the base refresh manager
  public static final boolean kUseBaseRefreshManager = false;

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
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

  public static final boolean kUseComponents = true;

  // set to false to disable alerts
  public static final boolean kUseAlerts = true && kCurrentMode != Mode.SIM;

  public static final class DriveConstants {
    public static final double kMaxLinearSpeed = 4.5; // meters per second
    // Tommy - this constant is unused right now, see my comment in driveToPointPeriodic
    public static final double kMaxDriveToPointSpeed = 3.6;
    // Tommy - delete these two constants after resolving the comment above
    public static final double kMaxAutoscoreSpeed = 3.6;
    public static final double kMaxAutoIntakeSpeed = 4.0;
    public static final double kMaxMeshedSpeed = 4.5;
    public static final double kMaxLinearAcceleration = 3.0; // meters per second squared
    public static final double kTrackWidthX = Units.inchesToMeters(22.75);
    public static final double kTrackWidthY = Units.inchesToMeters(22.75);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;
    public static final double kMaxAngularAcceleration = kMaxLinearAcceleration / kDriveBaseRadius;
    public static final LoggedTunableNumber kTeleopRotationSpeed =
        new LoggedTunableNumber("Teleop Rotation Speed", 10.0);
    public static final double kVisionSpeedConstantK = 0.5;

    // the exponent to raise the input to
    // ex 1.0 is linear, 2.0 is squared, etc
    public static final LoggedTunableNumber kDriveControlsExponent =
        new LoggedTunableNumber("Drive Control Mode", 2.0);

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

    public static final double kWheelRadius = Units.inchesToMeters(1.917);
    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveGearRatio =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // L2 gear ratio
    public static final double kTurnGearRatio = 150.0 / 7.0;

    // Simulation constants
    public static final double kDriveSimGearRatio = kDriveGearRatio;
    public static final double kDriveSimMOI = 0.025;
    public static final double kTurnSimGearRatio = kTurnGearRatio;
    public static final double kTurnSimMOI = 0.004;

    // universal reversals for drive (aka the big negative sign)
    // Tommy - if you find yourself needing to enable either of these, most likely your controls are
    // wrong or your modules are ID'd wrong
    public static final boolean kRealReversed = false;
    public static final boolean kSimReversed = false;

    public static final LoggedTunableNumber kDriveToPointP =
        new LoggedTunableNumber("DriveToPoint P", 3.2);
    public static final LoggedTunableNumber kDriveToPointI =
        new LoggedTunableNumber("DriveToPoint I", 0.0);
    public static final LoggedTunableNumber kDriveToPointD =
        new LoggedTunableNumber("DriveToPoint D", 0.18);

    public static final LoggedTunableNumber kDriveToPointAutoP =
        new LoggedTunableNumber("DriveToPoint Auto P", 3.0);
    public static final LoggedTunableNumber kDriveToPointAutoI =
        new LoggedTunableNumber("DriveToPoint Auto I", 0.0);
    public static final LoggedTunableNumber kDriveToPointAutoD =
        new LoggedTunableNumber("DriveToPoint Auto D", 0.12);

    public static final LoggedTunableNumber kDriveToPointHeadingP =
        new LoggedTunableNumber("DriveToPoint Heading P", 4.0);
    public static final LoggedTunableNumber kDriveToPointHeadingI =
        new LoggedTunableNumber("DriveToPoint Heading I", 0.0);
    public static final LoggedTunableNumber kDriveToPointHeadingD =
        new LoggedTunableNumber("DriveToPoint Heading D", 0.05);

    public static final LoggedTunableNumber kDriveToPointMaxVelocity =
        new LoggedTunableNumber("DriveToPoint Max Velocity", 3.0);
    public static final LoggedTunableNumber kDriveToPointMaxAcceleration =
        new LoggedTunableNumber("DriveToPoint Max Acceleration", 4.0);
    public static final LoggedTunableNumber kDriveToPointMaxDeceleration =
        new LoggedTunableNumber("DriveToPoint Max Deceleration", 3.0);

    public static final LoggedTunableNumber kMeshedXYP =
        new LoggedTunableNumber("Drive Meshed XY P", 3.0);
    public static final LoggedTunableNumber kMeshedXYD =
        new LoggedTunableNumber("Drive Meshed XY D", 0.12);
    public static final LoggedTunableNumber kMeshedThetaP =
        new LoggedTunableNumber("Drive Meshed Theta P", 3.0);
    public static final LoggedTunableNumber kMeshedThetaD =
        new LoggedTunableNumber("Drive Meshed Theta D", 0.0);
    public static final LoggedTunableNumber kDebounceAmount =
        new LoggedTunableNumber("Meshed Drive Debounce", 0.1);
    public static final LoggedTunableNumber kMeshDrivePriority =
        new LoggedTunableNumber("Meshed Drive Priority", 0.3);

    public static final LoggedTunableNumber kAutoscoreDeployDistance =
        new LoggedTunableNumber("Autoscore Deploy Distance", 42.0);
    public static final LoggedTunableNumber kAutoscoreOuttakeDistance =
        new LoggedTunableNumber("Autoscore Outtake Distance", 1.25);
    public static final LoggedTunableNumber kAutoscoreL1OuttakeDistance =
        new LoggedTunableNumber("Autoscore L1 Outtake Distance", 18.0);
    public static final LoggedTunableNumber kBargeScoreThrowDistance =
        new LoggedTunableNumber("Barge Score Throw Distance", 10.0);
    public static final LoggedTunableNumber kLoaderStationTimeout =
        new LoggedTunableNumber("Loader Station Timeout", 0.25);
    public static final LoggedTunableNumber kAutoAutoscoreTimeout =
        new LoggedTunableNumber("Auto Autoscore Timeout", 2.0);

    // radians per second squared to be considered slipping
    public static final LoggedTunableNumber kSlipThreshold =
        new LoggedTunableNumber("Slip Threshold", 150000);

    // meters per second squared to be considered in freefall (less than)
    public static final double kFreefallAccelerationThreshold = 9.0;

    public static final Mass kRobotMass = Pounds.of(120);
    public static final MomentOfInertia kRobotMOI = KilogramSquareMeters.of(6.5);
  }

  public static final class ClimbConstants {

    public static final LoggedTunableNumber kClimbP = new LoggedTunableNumber("Climb P", 30.0);
    public static final LoggedTunableNumber kClimbI = new LoggedTunableNumber("Climb I", 0.0);
    public static final LoggedTunableNumber kClimbD = new LoggedTunableNumber("Climb D", 0.0);
    public static final double kClimbTolerance = 5.0; // degrees

    public static final LoggedTunableNumber kStowFeedforward =
        new LoggedTunableNumber("Climb Stow Feedforward", 0.0);

    // these are magic units
    public static final LoggedTunableNumber kClimbStowPos =
        new LoggedTunableNumber("Climb Stow Angle", -225.0);
    public static final LoggedTunableNumber kClimbMatchPos =
        new LoggedTunableNumber("Climb Match Angle", 300.0);
    public static final LoggedTunableNumber kClimbDeployPos =
        new LoggedTunableNumber("Climb Deploy Angle", 600.0);

    public static final double kClimbReduction = (5.0 / 1.0) * (4.0 / 1.0) * (68.0 / 18.0);

    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(-350.0);
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(600.0);

    // sim constants
    public static final double kSimGearing = 1.0;
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(1);
    public static final double kSimClimbArmLengthMeters = 0.25;
    public static final double kSimMOI = 2.0;
    public static final double kSimStartingAngleRad = Units.degreesToRadians(0);
    public static final boolean kSimGravity = false;
    public static final double kSimClimbP = 0.5;
    public static final double kSimClimbI = 0.0;
    public static final double kSimClimbD = 0.35;
  }

  public static final class LedConstants {
    public static final int kStripLength = 22;

    public static final Color kOff = Color.kBlack;
    public static final Color kL1 = Color.kBlue;
    public static final Color kL2 = Color.kRed;
    public static final Color kL3 = Color.kYellow;
    public static final Color kL4 = Color.kGreen;
    public static final Color kDisabled = Color.kMagenta;
    public static final Color kLocationCheckYaw = Color.kDarkGreen;
    public static final Color kLocationCheckDistance = Color.kLightBlue;
    public static final Color kAlert = Color.kRed;
    public static final Color kFullTuning = Color.kWhite;
    public static final Color kVisionOff = Color.kBlack;
    public static final Color kAutoscoreMeasurementsBad = Color.kOrange;
    public static final Color kAutoscoreMeasurementsGood = Color.kDarkOliveGreen;
    public static final Color kHasGamePiece = Color.kBrown;
    public static final Color kIndexed = Color.kGreen;
    public static final Color kNotIndexed = Color.kRed;
  }

  public static final class ElevatorConstants {
    public static final LoggedTunableNumber kP0 = new LoggedTunableNumber("Elevator P0", 2.0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator I", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator D", 0.08);
    public static final LoggedTunableNumber kKG0 = new LoggedTunableNumber("Elevator kG0", 0.18);

    public static final LoggedTunableNumber kP1 = new LoggedTunableNumber("Elevator P1", 2.0);
    public static final LoggedTunableNumber kKG1 = new LoggedTunableNumber("Elevator kG1", 0.28);

    public static final LoggedTunableNumber kP2 = new LoggedTunableNumber("Elevator P2", 2.0);
    public static final LoggedTunableNumber kKG2 = new LoggedTunableNumber("Elevator kG2", 0.40);

    public static final LoggedTunableNumber kMagicMotionCruiseVelocity =
        new LoggedTunableNumber("Elevator MagicMotion cruise velocity", 5.0);
    public static final LoggedTunableNumber kMagicMotionAcceleration =
        new LoggedTunableNumber("Elevator MagicMotion acceleration", 10.0);
    public static final LoggedTunableNumber kMagicMotionJerk =
        new LoggedTunableNumber("Elevator MagicMotion Jerk", 100.0);

    public static final LoggedTunableNumber kStowHeight =
        new LoggedTunableNumber("Elevator Stow Height", 0.0);
    public static final LoggedTunableNumber kIntakingHeight =
        new LoggedTunableNumber("Elevator Intake Height", 0.0);
    public static final LoggedTunableNumber kAlgaeDescoringIntialHeightL2 =
        new LoggedTunableNumber("Elevator Algae Descore Initial Height L2", 19.0);
    public static final LoggedTunableNumber kAlgaeDescoringFinalHeightL2 =
        new LoggedTunableNumber("Elevator Algae Descore Final Height L2", 23.5);
    public static final LoggedTunableNumber kAlgaeDescoringIntialHeightL3 =
        new LoggedTunableNumber("Elevator Algae Descore Initial Height L3", 29.5);
    public static final LoggedTunableNumber kAlgaeDescoringFinalHeightL3 =
        new LoggedTunableNumber("Elevator Algae Descore Final Height L3", 38.5);
    public static final LoggedTunableNumber kBargeScoreHeight =
        new LoggedTunableNumber("Elevator Barge Score Height", 72.5);
    public static final LoggedTunableNumber kAlgaeHoldHeight =
        new LoggedTunableNumber("Elevator Algae Hold Height", 14.5);
    public static final LoggedTunableNumber kAlgaeOuttakeHeight =
        new LoggedTunableNumber("Elevator Algae Outtake Height", 3.5);
    public static final LoggedTunableNumber kLollipopIntakeHeight =
        new LoggedTunableNumber("Elevator Lollipop Intake Height", 3.0);
    public static final LoggedTunableNumber kDriveUpHeight =
        new LoggedTunableNumber("Drive Up Height", 11.5);

    public static final LoggedTunableNumber kBargeThrowHeight =
        new LoggedTunableNumber("Elevator Barge Throw Height", 17.0);

    public static final double kDiameter = 2.256; // inches
    public static final double kGearRatio = 54.0 / 12.0;
    // public static final double kSensorToMechanismRatio = kDiameter * Math.PI / kGearRatio;
    public static final double kSensorToMechanismRatio = (kGearRatio / (kDiameter * Math.PI));
    // public static final double kSensorToMechanismRatio = 1.0;
    public static final LoggedTunableNumber kElevatorOffset =
        new LoggedTunableNumber("Elevator Offset", Units.inchesToMeters(0));

    // this is the more than the max amount that the belts will ever skip
    public static final double kMaxSkip = 2.0;
    public static final LoggedTunableNumber kSlamTime = new LoggedTunableNumber("Slam Time", 0.2);
    public static final LoggedTunableNumber kSlamVoltage =
        new LoggedTunableNumber("Slam Voltage", -3.0);

    public static final double kZeroVelocityThreshold = 0.03;
    public static final double kZeroCurrentTheshold = 10.0;

    // Simulation constants
    public static final double kSimGearing = kGearRatio;
    public static final double kSimMOI = .001;
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60(2);
    public static final double kMinHeight = 0;
    public static final double kMaxHeight = 73.5;
    public static final double kHeightTolerance = 0.25;
    public static final double kSimElevatorP = 0.5;
    public static final double kSimElevatorI = 0.0;
    public static final double kSimElevatorD = 0.0;
    public static final double kSimElevatorkG = 0.0;
  }

  public static final class FullTuningConstants {
    public static final boolean kFullTuningMode = false;

    public static final LoggedTunableNumber kElevatorSetpoint =
        new LoggedTunableNumber("Full Tuning Elevator Setpoint", 0.0);
    public static final LoggedTunableNumber kIntakePivotSetpoint =
        new LoggedTunableNumber("Full Tuning Intake Pivot Setpoint", 0.0);
    public static final LoggedTunableNumber kManipulatorWristSetpoint =
        new LoggedTunableNumber("Full Tuning Manipulator Wrist Setpoint", 0.0);
    public static final LoggedTunableNumber kManipulatorRollerVoltage =
        new LoggedTunableNumber("Full Tuning Manipulator Roller Voltage", 0.0);
    public static final LoggedTunableNumber kIndexerVoltage =
        new LoggedTunableNumber("Full Tuning Indexer Voltage", 0.0);
  }

  public static final class AprilTagVisionConstants {
    public static final LoggedTunableNumber kUseVision = new LoggedTunableNumber("Use Vision", 1);

    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);

    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    // z margin is much smaller because the camera should be perfectly level with the tag
    public static final double kZMargin = 0.75;

    public static final double kErrorStandardDeviationThreshold = 0.2; // acceptable error

    public static final double kGyroAccurary =
        3.0; // higher numbers means the less we trust the vision/gyro sensor fusion

    // how long (sec) before we are considered disconnected
    public static final double kDisconnectTimeout = 5.0;

    public static final double kOdometryTrustFactorSlip = 0.2;
    public static final double kOdometryTrustFactorVisionScalar = 0.001;
    public static final double kOdometryTrustFactorNoVision = 0.04;

    public static final LoggedTunableNumber kXYStandardDeviationCoefficient =
        new LoggedTunableNumber("xyStandardDeviationCoefficient", 0.01);
    public static final LoggedTunableNumber kCloseStandardDeviation =
        new LoggedTunableNumber("AprilTagVision Close Standard Deviation", 0.01);
    public static final LoggedTunableNumber kFarStandardDeviation =
        new LoggedTunableNumber("AprilTagVision Far Standard Deviation", 0.01);
    public static final LoggedTunableNumber kThetaStandardDeviationCoefficient =
        new LoggedTunableNumber("thetaStandardDeviationCoefficient", 0.03);

    // transform from center of robot to camera
    public static final Transform3d[] kCameraTransforms =
        // NEW:
        // 0 - front right facing out, 10.4.22.229:1182, camera 3
        // 1 - front left facing out, 10.4.22.228:1182, camera 6
        // 2 - front left facing in, 10.4.22.229:1181, camera 11
        // 3 - front right facing in, 10.4.22.228:1181, camera 5
        new Transform3d[] {
          // front right (facing out)
          new Transform3d(
              Inches.of(9.796),
              Inches.of(-10.354),
              Inches.of(8.746),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-15.0), Degrees.of(-12.218))),

          // front left (facing out)
          new Transform3d(
              Inches.of(9.796),
              Inches.of(10.354),
              Inches.of(8.746),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-15.0), Degrees.of(12.218))),

          // front left (facing in)
          new Transform3d(
              Inches.of(10.484),
              Inches.of(11.464),
              Inches.of(8.771),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-17.43), Degrees.of(-19.0))),

          // front right (facing in)
          new Transform3d(
              Inches.of(10.484),
              Inches.of(-11.464),
              Inches.of(8.771),
              GeomUtil.constructRotation3d(Degrees.zero(), Degrees.of(-17.43), Degrees.of(19.0))),
        };

    // tolerances for using the vision rotation, temp values
    public static final double kRotationErrorThreshold = 0.3;
    public static final double kRotationDistanceThreshold = Units.inchesToMeters(200);
    public static final double kRotationSpeedThreshold = 0.2; // m/s
  }

  public static final class IntakeConstants {
    public static final double kPivotGearRatio = (72.0 / 8.0) * (36.0 / 14.0);
    public static final double kPivotAbsoluteEncoderGearRatio = 36.0 / 14.0;
    public static final double kRollerGearRatio = (30.0 / 12.0);
    public static final double kRollerRadius = Units.inchesToMeters(1.5);

    public static final LoggedTunableNumber kPivotTolerance =
        new LoggedTunableNumber("Pivot Tolerance", 65.0); // degrees

    // the offset needs to be so that it starts at 90 degrees (top)
    // public static final Rotation2d kPivotOffset =
    //     Rotation2d.fromDegrees(87.451171875).plus(Rotation2d.fromDegrees(90.0));
    // public static final Rotation2d kPivotOffset = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kPivotOffset =
        Rotation2d.fromDegrees(
            (24.0 + 134.0 + 8.0 + 16.1 + 3.0 + 9.0 - 6.0 - 2.0) * kPivotAbsoluteEncoderGearRatio);

    public static final Rotation2d kPivotFakeToRealOffset = Rotation2d.fromDegrees(75.0);

    public static final LoggedTunableNumber kZeroEncoderThreshold =
        new LoggedTunableNumber("Pivot Zero Threshikd", 122.0); // degrees

    public static final Angle kZeroEncoderValue = Degrees.of(137.0);

    // current to be considered holding a game piece
    // unused rn
    public static final double kRollerCoralCurrentThreshold = 99.0;

    public static final double kRollerCoralCurrentDebounce = 0.2; // seconds
    // L1 SETUP
    public static final LoggedTunableNumber kPivotP0 = new LoggedTunableNumber("Pivot P0", 16.0);
    // PASSTHROUGH SETUP
    // public static final LoggedTunableNumber kPivotP0 = new LoggedTunableNumber("Pivot P0", 20.0);
    public static final LoggedTunableNumber kPivotI = new LoggedTunableNumber("Pivot I", 0.0);
    public static final LoggedTunableNumber kPivotD0 = new LoggedTunableNumber("Pivot D0", 0.2);
    public static final LoggedTunableNumber kPivotP1 = new LoggedTunableNumber("Pivot P1", 30.0);
    public static final LoggedTunableNumber kPivotD1 = new LoggedTunableNumber("Pivot D1", 0.2);
    public static final LoggedTunableNumber kPivotP2 = new LoggedTunableNumber("Pivot P2", 30.0);
    public static final LoggedTunableNumber kPivotD2 = new LoggedTunableNumber("Pivot D2", 0.2);
    public static final LoggedTunableNumber kPivotKS = new LoggedTunableNumber("Pivot kS", 0.23);

    public static final LoggedTunableNumber kPivotStowAngle =
        new LoggedTunableNumber("Pivot Stow Angle", 30.0);
    public static final LoggedTunableNumber kPivotIntakeAngle =
        new LoggedTunableNumber("Pivot Intake Angle", 139.0);
    public static final LoggedTunableNumber kPivotHoldAngle =
        new LoggedTunableNumber("Pivot Hold Angle", 30.0);
    public static final LoggedTunableNumber kPivotOuttakeAngle =
        new LoggedTunableNumber("Pivot Outtake Angle", 48.0);
    public static final LoggedTunableNumber kPivotFunnelIntakeAngle =
        new LoggedTunableNumber("Pivot Funnel Intake Angle", 64.0);
    public static final LoggedTunableNumber kPivotCoralRunThroughAngle =
        new LoggedTunableNumber("Pivot Coral Run Through Angle", 30.0);
    public static final LoggedTunableNumber kPivotClimbAngle =
        new LoggedTunableNumber("Pivot Climb Angle", 50.0);
    public static final LoggedTunableNumber kIntakingFeedforward =
        new LoggedTunableNumber("Pivot Intaking Feedforward", 0.0);

    public static final LoggedTunableNumber kZeroVelocityThreshold =
        new LoggedTunableNumber("Pivot Zero Velocity Threshold", 1.0);
    public static final LoggedTunableNumber kIntakingFeedforwardThreshold =
        new LoggedTunableNumber("Pivot Intaking Feedforward Threshold", 125.0);

    public static final LoggedTunableNumber kRollerStowVoltage =
        new LoggedTunableNumber("Intake Roller Stow Voltage", 0.0);
    public static final LoggedTunableNumber kRollerIntakeVoltage =
        new LoggedTunableNumber("Intake Roller Intake Voltage", 3.5);
    public static final LoggedTunableNumber kRollerL1IntakeVoltage =
        new LoggedTunableNumber("Intake Roller L1 Intake Voltage", 3.0);
    public static final LoggedTunableNumber kRollerHoldVoltage =
        new LoggedTunableNumber("Intake Roller Hold Voltage", 0.29);
    public static final LoggedTunableNumber kRollerOuttakeVoltage =
        new LoggedTunableNumber("Intake Roller Outtake Voltage", -2.0);
    public static final LoggedTunableNumber kRollerFunnelIntakeVoltage =
        new LoggedTunableNumber("Intake Roller Funnel Intake Voltage", -2.0);
    public static final LoggedTunableNumber kRollerCoralRunThroughVoltage =
        new LoggedTunableNumber("Intake Roller Coral Run Through Voltage", 4.0);
    public static final LoggedTunableNumber kRollerEjectVoltage =
        new LoggedTunableNumber("Intake Roller Coral Eject Voltage", -3.0);

    public static final LoggedTunableNumber kCoralRunThroughTimeout =
        new LoggedTunableNumber("Coral Runthrough Timeout", 0.5);
    public static final LoggedTunableNumber kCoralRaiseTimeout =
        new LoggedTunableNumber("Coral Raise Timeout", 0.1);

    // Simulation constants
    public static final DCMotor kPivotSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kPivotSimGearing = kPivotGearRatio;
    public static final double kPivotArmMass = Units.lbsToKilograms(3.419);
    public static final double kPivotArmLength = Units.inchesToMeters(17.5);
    public static final double kPivotSimMOI =
        SingleJointedArmSim.estimateMOI(kPivotArmLength, kPivotArmMass);
    public static final Rotation2d kPivotMinAngle = Rotation2d.fromDegrees(30.0);
    public static final Rotation2d kPivotMaxAngle = Rotation2d.fromDegrees(140.0);
    public static final boolean kSimSimulateGravity = false;
    public static final Rotation2d kSimStartingAngle =
        Rotation2d.fromDegrees(Math.random() * kPivotMaxAngle.getDegrees());

    public static final DCMotor kRollerSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kRollerSimGearing = kRollerGearRatio;
    public static final double kRollerSimMOI = 0.004;

    public static final double kPivotSimP = .15;
    public static final double kPivotSimI = 0.0;
    public static final double kPivotSimD = 0.0;
  }

  public static final class ManipulatorConstants {
    public static final double kWristGearRatio = (66.0 / 10.0) * (32.0 / 14.0);
    // public static final double kWristGearRatio = 1.0;
    public static final double kWristAbsoluteEncoderGearRatio = 32.0 / 14.0;
    public static final double kRollerGearRatio = (58.0 / 16.0);
    public static final double kRollerRadius = Units.inchesToMeters(3);

    public static final double kWristTolerance = 2.0; // degrees

    // the offset needs to be so that it starts at 180 degrees (all the way out)
    // public static final Rotation2d kWristOffset =
    //     Rotation2d.fromDegrees(-78.662).plus(Rotation2d.fromDegrees(180.0));
    public static final Rotation2d kWristOffset =
        Rotation2d.fromDegrees((-37.4) * kWristAbsoluteEncoderGearRatio);
    // get owned shrihari - sri b is a better coder
    // public static final Rotation2d kWristOffset = Rotation2d.fromDegrees(0.0);

    public static final double kRollerPositionTolerance = 10.0; // degrees
    public static final LoggedTunableNumber kRollerFeedErrorTolerance =
        new LoggedTunableNumber("Manipulator Roller Feed Error Tolerance", 75.0);

    // how many degrees to move after photoelectric is tripped
    public static final LoggedTunableNumber kRollerIndexingPosition =
        new LoggedTunableNumber("Manipulator Roller Indexing Position", 150.0);

    public static final LoggedTunableNumber kWristP = new LoggedTunableNumber("Wrist P", 55.0);
    public static final LoggedTunableNumber kWristI = new LoggedTunableNumber("Wrist I", 0.0);
    public static final LoggedTunableNumber kWristD = new LoggedTunableNumber("Wrist D", 0.3);
    public static final LoggedTunableNumber kWristKS = new LoggedTunableNumber("Wrist kS", 0.25);

    public static final LoggedTunableNumber kWristStowAngle =
        new LoggedTunableNumber("Wrist Stow Angle", 100.0);
    public static final LoggedTunableNumber kWristIntakeAngle =
        new LoggedTunableNumber("Wrist Intake Angle", 100.0);
    public static final LoggedTunableNumber kWristScoringOffset =
        new LoggedTunableNumber("Wrist Scoring Offset", 0.0);
    public static final LoggedTunableNumber kWristAlgaeDescoringAngle =
        new LoggedTunableNumber("Wrist Algae Descoring Angle", 30.0);
    public static final LoggedTunableNumber kWristAlgaeHoldAngle =
        new LoggedTunableNumber("Wrist Algae Hold Angle", 35.0);
    public static final LoggedTunableNumber kWristAlgaeOuttakeAngle =
        new LoggedTunableNumber("Wrist Algae Outtake Angle", 0.0);

    public static final LoggedTunableNumber kRollerStowVoltage =
        new LoggedTunableNumber("Manipulator Roller Stow Voltage", 0.0);
    public static final LoggedTunableNumber kRollerIntakeVoltage =
        new LoggedTunableNumber("Manipulator Roller Intake Voltage", 2.0);
    public static final LoggedTunableNumber kRollerIndexingVoltage =
        new LoggedTunableNumber("Manipulator Roller Indexing Voltage", 1.0);
    public static final LoggedTunableNumber kRollerEjectVoltage =
        new LoggedTunableNumber("Manipulator Roller Eject Voltage", -3.0);
    public static final LoggedTunableNumber kRollerL1ScoringVoltageAutoscore =
        new LoggedTunableNumber("Manipulator Roller L1 Scoring Voltage Autoscore", 3.5);
    public static final LoggedTunableNumber kRollerL1ScoringVoltageManual =
        new LoggedTunableNumber("Manipulator Roller L1 Scoring Voltage Manual", 2.25);
    public static final LoggedTunableNumber kRollerL2L3ScoringVoltage =
        new LoggedTunableNumber("Manipulator Roller L2 L3 Scoring Voltage", 3.0);
    public static final LoggedTunableNumber kRollerL4ScoringVoltage =
        new LoggedTunableNumber("Manipulator Roller L4 Scoring Voltage", 3.0);
    public static final LoggedTunableNumber kRollerAlgaeDescoringVoltage =
        new LoggedTunableNumber("Manipulator Roller Algae Descoring Voltage", 3.0);
    public static final LoggedTunableNumber kRollerAlgaeHoldVoltage =
        new LoggedTunableNumber("Manipulator Roller Algae Hold Voltage", 1.0);
    public static final LoggedTunableNumber kRollerAlgaeOuttakeVoltage =
        new LoggedTunableNumber("Manipulator Roller Algae Outtake Voltage", -4.0);
    public static final LoggedTunableNumber kRollerBargeVoltage =
        new LoggedTunableNumber("Manipulator Roller Barge Voltage", -12.0);

    public static final LoggedTunableNumber kRollerP =
        new LoggedTunableNumber("Manipulator Roller P", 20.0);
    public static final LoggedTunableNumber kRollerI =
        new LoggedTunableNumber("Manipulator Roller I", 0.0);
    public static final LoggedTunableNumber kRollerD =
        new LoggedTunableNumber("Manipulator Roller D", 0.0);
    public static final LoggedTunableNumber kRollerKS =
        new LoggedTunableNumber("Manipulator Roller kS", 0.22);

    public static final LoggedTunableNumber kCoralOuttakeTimeout =
        new LoggedTunableNumber("Coral Outtake Timeout", 0.15);

    // Simulation constants
    public static final DCMotor kWristSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kWristSimGearing = kWristGearRatio;
    public static final double kWristArmMass = Units.lbsToKilograms(3.373);
    public static final double kWristArmLength = Units.inchesToMeters(7.75);
    public static final double kWristSimMOI =
        SingleJointedArmSim.estimateMOI(kWristArmLength, kWristArmMass);
    public static final Rotation2d kWristMinAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d kWristMaxAngle = Rotation2d.fromDegrees(130.0);
    public static final boolean kSimSimulateGravity = false;
    public static final Rotation2d kSimStartingAngle = Rotation2d.fromDegrees(Math.random() * 130);
    public static final double kSimRollerP = 0.01;
    public static final double kSimRollerI = 0.0;
    public static final double kSimRollerD = 0.0;
    public static final double kSimWristP = 0.150000;
    public static final double kSimWristI = 0.0;
    public static final double kSimWristD = 0.0;

    public static final DCMotor kRollerSimGearbox = DCMotor.getKrakenX60Foc(1);
    public static final double kRollerSimGearing = kRollerGearRatio;
    public static final double kRollerSimMOI = 0.004;
  }

  public static final class CurrentLimitConstants {
    // Drive
    public static final double kDriveDefaultSupplyCurrentLimit = 75.0;
    public static final double kDriveDefaultStatorCurrentLimit = 180.0;

    public static final double kTurnDefaultSupplyCurrentLimit = 60.0;
    public static final double kTurnDefaultStatorCurrentLimit = 120.0;

    // Intake
    public static final double kIntakePivotDefaultSupplyLimit = 80.0;
    public static final double kIntakePivotDefaultStatorLimit = 120.0;

    public static final double kIntakeRollerDefaultSupplyLimit = 80.0;
    public static final double kIntakeRollerDefaultStatorLimit = 120.0;

    // Indexer
    public static final double kIndexerDefaultSupplyLimit = 30.0;
    public static final double kIndexerDefaultStatorLimit = 120.0;

    // Manipulator
    public static final double kManipulatorWristDefaultSupplyLimit = 80.0;
    public static final double kManipulatorWristDefaultStatorLimit = 120.0;

    public static final double kManipulatorRollerDefaultSupplyLimit = 80.0;
    public static final double kManipulatorRollerDefaultStatorLimit = 120.0;

    // Elevator
    public static final double kElevatorDefaultSupplyLimit = 65.0;
    public static final double kElevatorDefaultStatorLimit = 95.0;

    // Climb
    public static final double kClimbDefaultSupplyLimit = 80.0;
    public static final double kClimbDefaultStatorLimit = 120.0;
  }

  public static final class IndexerConstants {

    public static final double kGearRatio = 30.0 / 8.0;
    public static final double kRollerRadius = Units.inchesToMeters(2);

    public static final LoggedTunableNumber kIndexerIdleVoltage =
        new LoggedTunableNumber("Indexer Idle Voltage", 0.0);
    public static final LoggedTunableNumber kIndexerIndexingVoltage =
        new LoggedTunableNumber("Indexer Indexing Voltage", 3.71);
    public static final LoggedTunableNumber kIndexerTopIndexingVoltage =
        new LoggedTunableNumber("Indexer Top Indexing Voltage", 2.5);
    public static final LoggedTunableNumber kIndexerEjectVoltage =
        new LoggedTunableNumber("Indexer Eject Voltage", -5.0);
    public static final LoggedTunableNumber kIndexerTopEjectVoltage =
        new LoggedTunableNumber("Indexer Top Eject Voltage", -2.5);

    // Simulation constants
    public static final DCMotor kSimGearbox = DCMotor.getKrakenX60Foc(2);
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

    public static final int kClimbMotor = 36;

    public static final int kIntakeRoller = 45;
    public static final int kIntakePivot = 46;
    public static final int kIntakeAbsoluteEncoder = 9;

    public static final int kManipulatorRoller = 29;
    public static final int kManipulatorWrist = 30;
    public static final int kManipulatorAbsoluteEncoder = 8;
    public static final int kManipulatorPhotoElectricOne = 6;
    public static final int kManipulatorPhotoElectricTwo = 7;
    public static final int kFunnelPhotoElectricOne = 4;
    public static final int kFunnelPhotoElectricTwo = 5;

    public static final int kIndexerSideMotor = 25;
    public static final int kIndexerTopMotor = 26;

    public static final int kLed = 9;

    public static final int kElevatorLead = 23;
    public static final int kElevatorFollowing = 24;

    public static final String kMainCanivoreName = "Main";
  }

  /** Whether or not subsystems are enabled on the proto board */
  public static final class ProtoConstants {
    public static final boolean kRealDrive = true;
    public static final boolean kRealIntake = true;
    public static final boolean kRealIndexer = true;
    public static final boolean kRealManipulator = true;
    public static final boolean kRealClimb = false;
    public static final boolean kRealElevator = false;
  }

  public static final class FieldConstants {
    public static final FieldType kFieldType = FieldType.WELDED;

    public static final double kFieldLength =
        AprilTagVisionConstants.kAprilTagLayout.getFieldLength();
    public static final double kFieldWidth =
        AprilTagVisionConstants.kAprilTagLayout.getFieldWidth();
    public static final double kStartingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double kAlgaeDiameter = Units.inchesToMeters(16);
    public static final double kCoralDiameter = Units.inchesToMeters(4.5);

    public static class Processor {
      public static final Pose2d kCenterFace =
          new Pose2d(
              AprilTagVisionConstants.kAprilTagLayout.getTagPose(16).get().getX(),
              0,
              Rotation2d.fromDegrees(90));
      public static final Pose2d kOpposingCenterFace =
          new Pose2d(
              AprilTagVisionConstants.kAprilTagLayout.getTagPose(3).get().getX(),
              kFieldWidth,
              Rotation2d.fromDegrees(-90));
    }

    public static class Barge {
      public static final double kNetWidth = Units.inchesToMeters(40.0);
      public static final double kNetHeight = Units.inchesToMeters(88.0);

      public static final double kCageWidth = Units.inchesToMeters(6.0);
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
      public static final double kStationLength = Units.inchesToMeters(79.750);
      public static final Pose2d kRightCenterFace =
          new Pose2d(
              Units.inchesToMeters(33.526),
              Units.inchesToMeters(25.824),
              Rotation2d.fromDegrees(144.011 - 90));
      public static final Pose2d kLeftCenterFace =
          new Pose2d(
              kRightCenterFace.getX(),
              kFieldWidth - kRightCenterFace.getY(),
              Rotation2d.fromRadians(-kRightCenterFace.getRotation().getRadians()));
    }

    public static class Reef {
      public static final double kFaceLength = Units.inchesToMeters(36.792600);
      public static final Translation2d kCenter =
          new Translation2d(Units.inchesToMeters(176.746), kFieldWidth / 2.0);
      public static final double kFaceToZoneLine =
          Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

      public static final Pose2d[] kCenterFaces =
          new Pose2d[6]; // Starting facing the driver station in clockwise order
      public static final List<Map<ReefHeight, Pose3d>> branchPositions =
          new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
      public static final List<Map<ReefHeight, Pose2d>> branchPositions2d = new ArrayList<>();

      static {
        // Initialize faces
        var aprilTagLayout = AprilTagVisionConstants.kAprilTagLayout;
        kCenterFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
        kCenterFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
        kCenterFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
        kCenterFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
        kCenterFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
        kCenterFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

        // Initialize branch positions
        for (int face = 0; face < 6; face++) {
          Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
          Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
          Map<ReefHeight, Pose2d> fillRight2d = new HashMap<>();
          Map<ReefHeight, Pose2d> fillLeft2d = new HashMap<>();
          for (var level : ReefHeight.values()) {
            Pose2d poseDirection = new Pose2d(kCenter, Rotation2d.fromDegrees(180 - (60 * face)));
            double adjustX = Units.inchesToMeters(30.738);
            double adjustY = Units.inchesToMeters(6.469);

            var rightBranchPose =
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians()));
            var leftBranchPose =
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians()));

            fillRight.put(level, rightBranchPose);
            fillLeft.put(level, leftBranchPose);
            fillRight2d.put(level, rightBranchPose.toPose2d());
            fillLeft2d.put(level, leftBranchPose.toPose2d());
          }
          branchPositions.add(fillRight);
          branchPositions.add(fillLeft);
          branchPositions2d.add(fillRight2d);
          branchPositions2d.add(fillLeft2d);
        }
      }
    }

    public static class StagingPositions {
      // Measured from the center of the ice cream
      public static final double kSeparation = Units.inchesToMeters(72.0);
      public static final Translation2d[] kIceCreams = new Translation2d[3];

      static {
        for (int i = 0; i < 3; i++) {
          kIceCreams[i] =
              new Translation2d(
                  Units.inchesToMeters(48), kFieldWidth / 2.0 - kSeparation + kSeparation * i);
        }
      }
    }

    public enum ReefHeight {
      L1(0, Units.inchesToMeters(25.0), 0),
      L2(1, Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
      L3(2, Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
      L4(3, Units.inchesToMeters(72), -90);

      ReefHeight(int levelNumber, double height, double pitch) {
        this.levelNumber = levelNumber;
        this.height = height;
        this.pitch = pitch; // Degrees
      }

      public static ReefHeight fromLevel(int level) {
        return Arrays.stream(values())
            .filter(height -> height.ordinal() == level)
            .findFirst()
            .orElse(L4);
      }

      public final int levelNumber;
      public final double height;
      public final double pitch;
    }

    @RequiredArgsConstructor
    public enum FieldType {
      ANDYMARK,
      WELDED,
    }
  }
}
