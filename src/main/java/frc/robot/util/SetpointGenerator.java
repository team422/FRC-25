package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.littletonUtils.AllianceRotateUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants.Reef;
import frc.robot.Constants.FieldConstants.ReefHeight;
import java.util.ArrayList;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  private SetpointGenerator() {}

  private static ArrayList<Pose2d> m_leftPoses = new ArrayList<>();
  private static ArrayList<Pose2d> m_rightPoses = new ArrayList<>();

  static {
    ArrayList<Pose2d> leftPoles = new ArrayList<>();
    ArrayList<Pose2d> rightPoles = new ArrayList<>();

    int i = 0;
    for (Map<ReefHeight, Pose2d> poses : Reef.branchPositions2d) {
      if (i % 2 == 0) {
        rightPoles.add(poses.get(ReefHeight.L1));
      } else {
        leftPoles.add(poses.get(ReefHeight.L1));
      }
      i++;
    }

    System.out.println(leftPoles.size());

    for (Pose2d pole : leftPoles) {
      var pose =
          pole.transformBy(
              new Transform2d(
                  DriveConstants.kTrackWidthX, 0, new Rotation2d().rotateBy(Rotation2d.k180deg)));
      m_leftPoses.add(AllianceRotateUtil.apply(pose));
    }

    for (Pose2d pole : rightPoles) {
      var pose =
          pole.transformBy(
              new Transform2d(
                  DriveConstants.kTrackWidthX, 0, new Rotation2d().rotateBy(Rotation2d.k180deg)));
      m_rightPoses.add(AllianceRotateUtil.apply(pose));
    }
  }

  public static Pose2d getAutoscorePosition(Pose2d pose, boolean left) {
    Pose2d newest;
    Logger.recordOutput("left", left);
    if (left) {
      newest = pose.nearest(m_leftPoses);
    } else {
      newest = pose.nearest(m_rightPoses);
    }
    Logger.recordOutput("Autoscore/nearestPose", newest);
    return newest;
  }
}
