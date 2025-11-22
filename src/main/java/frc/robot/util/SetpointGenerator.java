package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.littletonUtils.AllianceRotateUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants.Reef;
import frc.robot.Constants.FieldConstants.ReefHeight;
import java.util.ArrayList;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  private SetpointGenerator() {}

  private static ArrayList<Pose2d> m_leftPoles = new ArrayList<>();
  private static ArrayList<Pose2d> m_rightPoles = new ArrayList<>();
  private static ArrayList<Pose2d> m_leftPoses = new ArrayList<>();
  private static ArrayList<Pose2d> m_rightPoses = new ArrayList<>();

  static {
    int i = 0;
    for (Map<ReefHeight, Pose2d> poses : Reef.branchPositions2d) {
      if (i % 2 == 0) {
        m_rightPoles.add(poses.get(ReefHeight.L1));
      } else {
        m_leftPoles.add(poses.get(ReefHeight.L1));
      }
      i++;
    }

    System.out.println(m_leftPoles.size());

    for (Pose2d pose : m_leftPoles) {
      m_leftPoses.add(AllianceRotateUtil.apply(
          new Pose2d(
              pose.getTranslation()
                  .plus(
                      new Translation2d(
                          DriveConstants.kTrackWidthX * Math.cos(pose.getRotation().getRadians()),
                          DriveConstants.kTrackWidthY * Math.sin(pose.getRotation().getRadians()))),
              pose.getRotation().rotateBy(Rotation2d.k180deg))));
    }

    for (Pose2d pose : m_rightPoles) {
      m_rightPoses.add(AllianceRotateUtil.apply(
          new Pose2d(
              pose.getTranslation()
                  .plus(
                      new Translation2d(
                          DriveConstants.kTrackWidthX * Math.cos(pose.getRotation().getRadians()),
                          DriveConstants.kTrackWidthY * Math.sin(pose.getRotation().getRadians()))),
              pose.getRotation().rotateBy(Rotation2d.k180deg))));
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
    Logger.recordOutput("nearest", newest);
    return newest;
  }
}
