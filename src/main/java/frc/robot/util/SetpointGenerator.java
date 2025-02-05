package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import java.util.Map;

public class SetpointGenerator {
  public static record RobotSetpoint(
      Pose2d drivePose, Rotation2d manipulatorAngle, double elevatorHeight) {}

  // TODO: populate with real values
  private static final Map<ReefHeight, Double> kElevatorHeights =
      Map.of(
          ReefHeight.L1, 0.0,
          ReefHeight.L2, 0.0,
          ReefHeight.L3, 0.0,
          ReefHeight.L4, 0.0);

  private static final Map<ReefHeight, Rotation2d> kManipulatorAngles =
      Map.of(
          ReefHeight.L1, Rotation2d.fromDegrees(0.0),
          ReefHeight.L2, Rotation2d.fromDegrees(0.0),
          ReefHeight.L3, Rotation2d.fromDegrees(0.0),
          ReefHeight.L4, Rotation2d.fromDegrees(0.0));

  /**
   * Generates a setpoint for the robot to score on the reef.
   *
   * @param reefIndex The index of the reef to score on. Must be in the range [0, 12), starting from
   *     the driver station in clockwise order.
   * @param reefHeight The height of the reef to score on. Using the reef height enum to avoid
   *     confusion with indices.
   * @return
   */
  public static RobotSetpoint generate(int reefIndex, ReefHeight reefHeight) {
    if (reefIndex < 0 || reefIndex >= FieldConstants.Reef.kCenterFaces.length) {
      throw new IllegalArgumentException("Invalid reef index");
    }
    if (reefHeight == null) {
      throw new IllegalArgumentException("Invalid reef height");
    }

    var branchPose = FieldConstants.Reef.kBranchPositions.get(reefIndex).get(reefHeight);

    var drivePose = branchPose.toPose2d();
    var manipulatorAngle = kManipulatorAngles.get(reefHeight);
    var elevatorHeight = kElevatorHeights.get(reefHeight);

    return new RobotSetpoint(drivePose, manipulatorAngle, elevatorHeight);
  }

  public static Pair<Integer, Integer> getPossibleIndices(Pose2d drivePose) {
    var centerPoses = FieldConstants.Reef.kCenterFaces;
    double minDistance = Double.MAX_VALUE;
    Pair<Integer, Integer> indices = new Pair<>(-1, -1);
    for (int i = 0; i < centerPoses.length; i++) {
      double currDistance = centerPoses[i].getTranslation().getDistance(drivePose.getTranslation());
      if (currDistance < minDistance) {
        indices = new Pair<>(i * 2, i * 2 + 1);
        minDistance = currDistance;
      }
    }
    return indices;
  }
}
