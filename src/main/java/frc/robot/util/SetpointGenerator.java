package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.littletonUtils.AllianceFlipUtil;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  // this is a static class and may not be instantiated
  private SetpointGenerator() {}

  public static record RobotSetpoint(
      Pose2d drivePose, Rotation2d manipulatorAngle, double elevatorHeight) {}

  public static record MeshedSetpoint(
      Pose2d desiredPose, double slope, double intercept, double minX, double maxX) {}

  private static final Map<ReefHeight, LoggedTunableNumber> kElevatorHeights =
      Map.of(
          ReefHeight.L1, new LoggedTunableNumber("Elevator L1 Height", 9.5),
          ReefHeight.L2, new LoggedTunableNumber("Elevator L2 Height", 27.0),
          ReefHeight.L3, new LoggedTunableNumber("Elevator L3 Height", 43.25),
          ReefHeight.L4, new LoggedTunableNumber("Elevator L4 Height", 71.0));

  private static final Map<ReefHeight, LoggedTunableNumber> kManipulatorAngles =
      Map.of(
          ReefHeight.L1, new LoggedTunableNumber("Wrist L1 Angle", 112.0),
          ReefHeight.L2, new LoggedTunableNumber("Wrist L2 Angle", 55.0),
          ReefHeight.L3, new LoggedTunableNumber("Wrist L3 Angle", 55.0),
          ReefHeight.L4, new LoggedTunableNumber("Wrist L4 Angle", 40.0));

  // we need to move back a bit from the raw branch pose
  private static final double kDriveXOffset =
      DriveConstants.kTrackWidthX / 2.0 + Units.inchesToMeters(6.5);

  private static final double kDriveXOffsetFinalAlgae =
      DriveConstants.kTrackWidthX / 2.0 + Units.inchesToMeters(25.0);

  private static final List<MeshedSetpoint> kIntakePositionsRed =
      List.of(
          new MeshedSetpoint(
              new Pose2d(16.32, 0.96, Rotation2d.fromDegrees(126)), .714, -10.50, 15.745, 16.73),
          new MeshedSetpoint(
              new Pose2d(16.32, 6.1, Rotation2d.fromDegrees(-126)), -.714, 18.55, 15.745, 16.73));
  private static final List<MeshedSetpoint> kIntakePositionsBlue =
      List.of(
          new MeshedSetpoint(
              new Pose2d(1.0, 0.96, Rotation2d.fromDegrees(54)), -.714, 1.98214, 0.7419, 1.7269),
          new MeshedSetpoint(
              new Pose2d(1.0, 6.1, Rotation2d.fromDegrees(-54)), .714, 6.0678634, 0.7419, 1.7269));

  // we need to move sideways to get from the center to the branch
  // this number is taken from the calculations done in FieldConstants (but it's not a constant)
  private static final double kDriveYOffset = Units.inchesToMeters(6.469);

  private static final double kDriveYL1Offset = Units.inchesToMeters(13.469);
  private static final Rotation2d kRotationL1 = Rotation2d.fromDegrees(10.0);

  private static final double kBargeXOffset = Units.inchesToMeters(48.0 + 28.0 / 2.0);

  private static final LoggedTunableNumber kElevatorL1Autoscore =
      new LoggedTunableNumber("Elevator L1 Autoscore Height", 11.5);
  private static final LoggedTunableNumber kManipulatorL1Autoscore =
      new LoggedTunableNumber("Wrist L1 Autoscore Angle", 95.0);

  // we are considered "close" to a field element (processor, barge) if we're within this distance
  private static final double kDistanceThreshold = Units.inchesToMeters(36.0);

  /**
   * Generates a setpoint for the robot to score on the reef.
   *
   * @param reefIndex The index of the reef to score on. Must be in the range [0, 12), starting from
   *     the driver station right side in clockwise order. Even indices are right, odd indices are
   *     left.
   * @param reefHeight The height of the reef to score on. Using the reef height enum to avoid
   *     confusion with indices.
   * @param autoScore Whether or not the robot is currently trying to autoscore. This changes the
   *     manipulator angle and elevator height for L1 to score on the side instead of the middle.
   * @return A RobotSetpoint object with the desired pose, manipulator angle, and elevator height.
   */
  public static RobotSetpoint generate(int reefIndex, ReefHeight reefHeight, boolean autoScore) {
    if (reefIndex < 0 || reefIndex >= FieldConstants.Reef.kCenterFaces.length * 2) {
      throw new IllegalArgumentException("Invalid reef index: " + reefIndex);
    }
    if (reefHeight == null) {
      throw new IllegalArgumentException("Invalid reef height: " + reefHeight);
    }

    var centerFacePose = AllianceFlipUtil.apply(FieldConstants.Reef.kCenterFaces[reefIndex / 2]);

    // we need to move away from the center of the reef (regardless of angle)
    var drivePoseFinal =
        centerFacePose.transformBy(
            new Transform2d(
                kDriveXOffset,
                // move to left or right depending on the reef index
                // if L1 then use L1
                (reefHeight == ReefHeight.L1)
                    ? kDriveYL1Offset * ((reefIndex % 2 == 0) ? 1 : -1)
                    : kDriveYOffset * ((reefIndex % 2 == 0) ? 1 : -1),
                (reefHeight == ReefHeight.L1)
                    ? Rotation2d.fromDegrees(180)
                        .plus(kRotationL1.times((reefIndex % 2 == 0) ? -1 : 1))
                    : Rotation2d.fromDegrees(180)));

    var manipulatorAngle = kManipulatorAngles.get(reefHeight).get();
    if (RobotState.getInstance() != null
        && reefHeight == ReefHeight.L1
        && (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoScore
            || RobotState.getInstance().getCurrentAction() == RobotAction.kAutoAutoScore)) {
      manipulatorAngle = kManipulatorL1Autoscore.get();
    }

    var elevatorHeight = kElevatorHeights.get(reefHeight).get();
    if (RobotState.getInstance() != null
        && reefHeight == ReefHeight.L1
        && (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoScore
            || RobotState.getInstance().getCurrentAction() == RobotAction.kAutoAutoScore)) {
      elevatorHeight = kElevatorL1Autoscore.get();
    }

    return new RobotSetpoint(
        drivePoseFinal, Rotation2d.fromDegrees(manipulatorAngle), elevatorHeight);
  }

  /**
   * Generates two possible reef indices to score on based on the robot's current pose. Picks based
   * on the closest reef center face.
   *
   * @param drivePose The robot's current pose.
   * @return A pair of reef indices that are closest to the robot's current pose. The first index is
   *     the right reef, and the second index is the left reef.
   */
  public static Pair<Integer, Integer> getPossibleIndices(Pose2d drivePose) {
    // i used desmos to figure out these equations
    // blue:
    // x = 4.503
    // y = sqrt3 / 3 * x + 1.4
    // y = -sqrt3 / 3 * x + 6.6
    // red:
    // x = 13.062
    // y = sqrt3 / 3 * x - 3.53
    // y = -sqrt3 / 3 * x + 11.55

    int[] vertical = checkVertical(drivePose);
    int[] diagonalPositive = checkDiagonalPositive(drivePose);
    int[] diagonalNegative = checkDiagonalNegative(drivePose);

    Logger.recordOutput("SetpointGenerator/Vertical", vertical);
    Logger.recordOutput("SetpointGenerator/DiagonalPositive", diagonalPositive);
    Logger.recordOutput("SetpointGenerator/DiagonalNegative", diagonalNegative);

    // find the common index
    int commonIndex = findCommonElement(vertical, diagonalPositive, diagonalNegative);

    // turn the common index into a pair of indices (0-5 becomes 0-11)
    int rightIndex = commonIndex * 2;
    int leftIndex = commonIndex * 2 + 1;

    Logger.recordOutput("SetpointGenerator/CommonIndex", commonIndex);
    Logger.recordOutput("SetpointGenerator/RightIndex", rightIndex);
    Logger.recordOutput("SetpointGenerator/LeftIndex", leftIndex);

    return Pair.of(rightIndex, leftIndex);
  }

  private static int findCommonElement(int[] arr1, int[] arr2, int[] arr3) {
    for (int num : arr1) {
      if (contains(arr2, num) && contains(arr3, num)) {
        return num; // Return the first common element found
      }
    }
    return -1; // No common element found
  }

  private static boolean contains(int[] arr, int target) {
    for (int num : arr) {
      if (num == target) {
        return true;
      }
    }
    return false;
  }

  private static int[] checkVertical(Pose2d pose) {
    // check the alliance
    Alliance alliance = getFieldAlliance(pose);
    if (alliance == Alliance.Red) {
      if (pose.getX() >= 13.062) {
        return new int[] {0, 1, 5};
      } else {
        return new int[] {2, 3, 4};
      }
    } else {
      if (pose.getX() <= 4.503) {
        return new int[] {0, 1, 5};
      } else {
        return new int[] {2, 3, 4};
      }
    }
  }

  private static int[] checkDiagonalPositive(Pose2d pose) {
    Alliance alliance = getFieldAlliance(pose);
    double m = Math.sqrt(3) / 3;
    double b = alliance == Alliance.Red ? -3.53 : 1.4;
    double ty = m * pose.getX() + b;
    if (pose.getY() >= ty) {
      if (alliance == Alliance.Red) {
        return new int[] {1, 2, 3};
      } else {
        return new int[] {0, 1, 2};
      }
    } else {
      if (alliance == Alliance.Red) {
        return new int[] {0, 4, 5};
      } else {
        return new int[] {3, 4, 5};
      }
    }
  }

  private static int[] checkDiagonalNegative(Pose2d pose) {
    Alliance alliance = getFieldAlliance(pose);
    double m = -Math.sqrt(3) / 3;
    double b = alliance == Alliance.Red ? 11.55 : 6.6;
    double ty = m * pose.getX() + b;
    if (pose.getY() >= ty) {
      if (alliance == Alliance.Red) {
        return new int[] {0, 1, 2};
      } else {
        return new int[] {1, 2, 3};
      }
    } else {
      if (alliance == Alliance.Red) {
        return new int[] {3, 4, 5};
      } else {
        return new int[] {0, 4, 5};
      }
    }
  }

  public static boolean isNearProcessor(Pose2d drivePose) {
    // no need for abs because of distance formula
    Pose2d processorPose = FieldConstants.Processor.kCenterFace;
    processorPose =
        AllianceFlipUtil.shouldFlip()
            ? processorPose.rotateAround(
                new Translation2d(
                    FieldConstants.kFieldLength / 2.0, FieldConstants.kFieldWidth / 2.0),
                Rotation2d.fromDegrees(180))
            : processorPose;
    return drivePose.getTranslation().getDistance(processorPose.getTranslation())
        < kDistanceThreshold;
  }

  public static boolean isNearBarge(Pose2d drivePose) {
    // for the barge we only care about the x coordinate
    return Math.abs(
            drivePose.getTranslation().getX()
                - AllianceFlipUtil.apply(FieldConstants.Barge.kMiddleCage.getX()))
        < kDistanceThreshold;
  }

  public static ReefHeight getAlgaeHeight(Pose2d drivePose) {
    // first we get the closest reef center face
    Pose2d closestCenterFace = FieldConstants.Reef.kCenterFaces[0];
    var fieldAlliance = getFieldAlliance(drivePose);
    if (fieldAlliance == Alliance.Red) {
      // flip the pose
      // i am very sorry for this eldritch horror but AllianceFlipUtil doesn't have an Alliance
      // parameter and i dont wanna add that to every single method
      var translation = closestCenterFace.getTranslation();
      var rotation = closestCenterFace.getRotation();
      closestCenterFace =
          new Pose2d(
              new Translation2d(
                  FieldConstants.kFieldLength - translation.getX(), translation.getY()),
              new Rotation2d(-rotation.getCos(), rotation.getSin()));
    }
    int closestIndex = 0;
    for (int i = 1; i < FieldConstants.Reef.kCenterFaces.length; i++) {
      Pose2d curr = FieldConstants.Reef.kCenterFaces[i];

      if (fieldAlliance == Alliance.Red) {
        // flip the pose
        // i am very sorry for this eldritch horror but AllianceFlipUtil doesn't have an Alliance
        // parameter and i dont wanna add that to every single method
        var translation = curr.getTranslation();
        var rotation = curr.getRotation();
        curr =
            new Pose2d(
                new Translation2d(
                    FieldConstants.kFieldLength - translation.getX(), translation.getY()),
                new Rotation2d(-rotation.getCos(), rotation.getSin()));
      }
      if (drivePose.getTranslation().getDistance(curr.getTranslation())
          < drivePose.getTranslation().getDistance(closestCenterFace.getTranslation())) {
        closestCenterFace = curr;
        closestIndex = i;
      }
    }
    Logger.recordOutput("A", closestCenterFace);
    // now we determine if the algae is at L2 or L3
    ReefHeight algaeHeight = closestIndex % 2 == 0 ? ReefHeight.L3 : ReefHeight.L2;
    return algaeHeight;
  }

  public static Pose2d generateAlgae(int algaeIndex, Pose2d robotPose) {
    Pose2d centerFacePose = FieldConstants.Reef.kCenterFaces[algaeIndex];
    var fieldAlliance = getFieldAlliance(robotPose);
    if (fieldAlliance == Alliance.Red) {
      // flip the pose
      // i am very sorry for this eldritch horror but AllianceFlipUtil doesn't have an Alliance
      // parameter and i dont wanna add that to every single method
      var translation = centerFacePose.getTranslation();
      var rotation = centerFacePose.getRotation();
      centerFacePose =
          new Pose2d(
              new Translation2d(
                  FieldConstants.kFieldLength - translation.getX(), translation.getY()),
              new Rotation2d(-rotation.getCos(), rotation.getSin()));
    }
    // we need to move away from the center of the reef (regardless of angle)
    var drivePoseFinal =
        centerFacePose.transformBy(
            new Transform2d(kDriveXOffset, 0.0, Rotation2d.fromDegrees(180)));
    return drivePoseFinal;
  }

  private static Alliance getFieldAlliance(Pose2d robotPose) {
    Logger.recordOutput("SetpointGenerator/AlliancePose", robotPose);

    var translation = robotPose.getTranslation();
    if (translation.getX() < FieldConstants.kFieldLength / 2.0) {
      Logger.recordOutput("SetpointGenerator/Alliance", Alliance.Blue);
      return Alliance.Blue;
    } else {
      Logger.recordOutput("SetpointGenerator/Alliance", Alliance.Red);
      return Alliance.Red;
    }
  }

  public static MeshedSetpoint generateNearestIntake(Pose2d curPose) {
    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
      if (curPose.getY() < FieldConstants.kFieldWidth / 2) {
        return kIntakePositionsRed.get(0);
      } else {
        return kIntakePositionsRed.get(1);
      }
    } else {
      if (curPose.getY() < FieldConstants.kFieldWidth / 2) {
        return kIntakePositionsBlue.get(0);
      } else {
        return kIntakePositionsBlue.get(1);
      }
    }
  }

  public static Pose2d generateAlgaeFinal(int algaeIndex, Pose2d robotPose) {
    Pose2d centerFacePose = FieldConstants.Reef.kCenterFaces[algaeIndex];
    var fieldAlliance = getFieldAlliance(robotPose);
    if (fieldAlliance == Alliance.Red) {
      // flip the pose
      // i am very sorry for this eldritch horror but AllianceFlipUtil doesn't have an Alliance
      // parameter and i dont wanna add that to every single method
      var translation = centerFacePose.getTranslation();
      var rotation = centerFacePose.getRotation();
      centerFacePose =
          new Pose2d(
              new Translation2d(
                  FieldConstants.kFieldLength - translation.getX(), translation.getY()),
              new Rotation2d(-rotation.getCos(), rotation.getSin()));
    }
    // we need to move away from the center of the reef (regardless of angle)
    var drivePoseFinal =
        centerFacePose.transformBy(
            new Transform2d(kDriveXOffsetFinalAlgae, 0.0, Rotation2d.fromDegrees(180)));
    return drivePoseFinal;
  }

  public static int getAlgaeIndex(Pose2d drivePose) {
    int[] vertical = checkVertical(drivePose);
    int[] diagonalPositive = checkDiagonalPositive(drivePose);
    int[] diagonalNegative = checkDiagonalNegative(drivePose);

    Logger.recordOutput("SetpointGenerator/Vertical", vertical);
    Logger.recordOutput("SetpointGenerator/DiagonalPositive", diagonalPositive);
    Logger.recordOutput("SetpointGenerator/DiagonalNegative", diagonalNegative);

    // find the common index
    int commonIndex = findCommonElement(vertical, diagonalPositive, diagonalNegative);

    return commonIndex;
  }

  public static Pose2d generateBargeLeft(Pose2d robotPose) {
    Pose2d res =
        new Pose2d(FieldConstants.Barge.kMiddleCage, new Rotation2d())
            .rotateAround(
                new Translation2d(
                    FieldConstants.kFieldLength / 2.0, FieldConstants.kFieldWidth / 2.0),
                Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 180 : 0))
            .transformBy(new Transform2d(-kBargeXOffset, 0.0, Rotation2d.fromDegrees(25)));
    // if our field alliance doesn't match our actual alliance we need to flip
    if (getFieldAlliance(robotPose) != DriverStation.getAlliance().orElse(Alliance.Red)) {
      // another eldritch horror
      res =
          new Pose2d(
              new Translation2d(FieldConstants.kFieldLength - res.getX(), res.getY()),
              new Rotation2d(-res.getRotation().getCos(), res.getRotation().getSin()));
    }
    return res;
  }

  public static Pose2d generateBargeRight(Pose2d robotPose) {
    Pose2d res =
        new Pose2d(FieldConstants.Barge.kCloseCage, new Rotation2d())
            .rotateAround(
                new Translation2d(
                    FieldConstants.kFieldLength / 2.0, FieldConstants.kFieldWidth / 2.0),
                Rotation2d.fromDegrees(AllianceFlipUtil.shouldFlip() ? 180 : 0))
            .transformBy(new Transform2d(-kBargeXOffset, 0.0, Rotation2d.fromDegrees(25)));
    // if our field alliance doesn't match our actual alliance we need to flip
    if (getFieldAlliance(robotPose) != DriverStation.getAlliance().orElse(Alliance.Red)) {
      // another eldritch horror
      res =
          new Pose2d(
              new Translation2d(FieldConstants.kFieldLength - res.getX(), res.getY()),
              new Rotation2d(-res.getRotation().getCos(), res.getRotation().getSin()));
    }
    return res;
  }

  public static Rotation2d generateLollipopAngle(Translation2d driveTranslation) {
    var lollipopPoses = new ArrayList<>(Arrays.asList(FieldConstants.StagingPositions.kIceCreams));

    lollipopPoses.replaceAll(AllianceFlipUtil::apply);

    var closestTranslation = lollipopPoses.get(0);

    for (var pose : lollipopPoses) {
      if (pose.getDistance(driveTranslation) < closestTranslation.getDistance(driveTranslation)) {
        closestTranslation = pose;
      }
    }

    double dy = closestTranslation.getY() - driveTranslation.getY();
    double dx = closestTranslation.getX() - driveTranslation.getX();

    return Rotation2d.fromRadians(Math.atan2(dy, dx));
  }

  public static void logLines() {
    List<Translation2d[]> values = new ArrayList<>();
    for (var pos : kIntakePositionsRed) {
      double y1 = pos.slope * pos.minX + pos.intercept;
      double y2 = pos.slope * pos.maxX + pos.intercept;
      Translation2d[] line = {new Translation2d(pos.minX, y1), new Translation2d(pos.maxX, y2)};
      values.add(line);
    }
    for (var pos : kIntakePositionsBlue) {
      double y1 = pos.slope * pos.minX + pos.intercept;
      double y2 = pos.slope * pos.maxX + pos.intercept;
      Translation2d[] line = {new Translation2d(pos.minX, y1), new Translation2d(pos.maxX, y2)};
      values.add(line);
    }
    for (int i = 0; i < values.size(); i++) {
      Logger.recordOutput("Lines/" + i, values.get(i));
    }
  }
}
