package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import java.util.Map;

public class SetpointGenerator {
  // this is a static class and may not be instantiated
  private SetpointGenerator() {}

  public static record RobotSetpoint(
      Pose2d drivePose, Rotation2d manipulatorAngle, double elevatorHeight) {}

  // TODO: populate with real values
  private static final Map<ReefHeight, Double> kElevatorHeights =
      Map.of(
          ReefHeight.L1, 0.0,
          ReefHeight.L2, 0.5,
          ReefHeight.L3, 1.0,
          ReefHeight.L4, 1.5);

  private static final Map<ReefHeight, Rotation2d> kManipulatorAngles =
      Map.of(
          ReefHeight.L1, Rotation2d.fromDegrees(0.0),
          ReefHeight.L2, Rotation2d.fromDegrees(30.0),
          ReefHeight.L3, Rotation2d.fromDegrees(60.0),
          ReefHeight.L4, Rotation2d.fromDegrees(80.0));

  // we need to move back a bit from the raw branch pose
  private static final double kDriveXOffset =
      DriveConstants.kTrackWidthX / 2.0 + Units.inchesToMeters(2.0);

  // we need to move sideways to get from the center to the branch
  // this number is taken from the calculations done in FieldConstants (but it's not a constant)
  private static final double kDriveYOffset = Units.inchesToMeters(6.469);

  // we are considered "close" to a field element (processor, barge) if we're within this distance
  private static final double kDistanceThreshold = Units.inchesToMeters(24.0);

  /**
   * Generates a setpoint for the robot to score on the reef.
   *
   * @param reefIndex The index of the reef to score on. Must be in the range [0, 12), starting from
   *     the driver station right side in clockwise order. Even indices are right, odd indices are
   *     left.
   * @param reefHeight The height of the reef to score on. Using the reef height enum to avoid
   *     confusion with indices.
   * @return A RobotSetpoint object with the desired pose, manipulator angle, and elevator height.
   */
  public static RobotSetpoint generate(int reefIndex, ReefHeight reefHeight) {
    if (reefIndex < 0 || reefIndex >= FieldConstants.Reef.kCenterFaces.length * 2) {
      throw new IllegalArgumentException("Invalid reef index: " + reefIndex);
    }
    if (reefHeight == null) {
      throw new IllegalArgumentException("Invalid reef height: " + reefHeight);
    }

    var centerFacePose = FieldConstants.Reef.kCenterFaces[reefIndex / 2];

    // we need to move away from the center of the reef (regardless of angle)
    var drivePoseFinal =
        centerFacePose.transformBy(
            new Transform2d(
                kDriveXOffset,
                // move to left or right depending on the reef index
                (reefIndex % 2 == 0) ? kDriveYOffset : -kDriveYOffset,
                new Rotation2d()));

    var manipulatorAngle = kManipulatorAngles.get(reefHeight);
    var elevatorHeight = kElevatorHeights.get(reefHeight);

    return new RobotSetpoint(drivePoseFinal, manipulatorAngle, elevatorHeight);
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
    var branchPoses = FieldConstants.Reef.kBranchPositions;
    double minLeftDistance = Double.POSITIVE_INFINITY;
    double minRightDistance = Double.POSITIVE_INFINITY;
    int leftIndex = -1;
    int rightIndex = -1;
    for (int i = 0; i < branchPoses.size(); i++) {
      // the reef height doesn't matter here so we just use L1
      Translation2d curr = branchPoses.get(i).get(ReefHeight.L1).toPose2d().getTranslation();
      double distance = drivePose.getTranslation().getDistance(curr);
      if (i % 2 == 0) {
        // left branch
        if (distance < minLeftDistance) {
          minLeftDistance = distance;
          leftIndex = i;
        }
      } else {
        // right branch
        if (distance < minRightDistance) {
          minRightDistance = distance;
          rightIndex = i;
        }
      }
    }
    return new Pair<>(rightIndex, leftIndex);
  }

  public static boolean isNearProcessor(Pose2d drivePose) {
    // no need for abs because of distance formula
    return drivePose
            .getTranslation()
            .getDistance(FieldConstants.Processor.kCenterFace.getTranslation())
        < kDistanceThreshold;
  }

  public static boolean isNearBarge(Pose2d drivePose) {
    // for the barge we only care about the x coordinate
    return Math.abs(drivePose.getTranslation().getX() - FieldConstants.Barge.kMiddleCage.getX())
        < kDistanceThreshold;
  }
}
