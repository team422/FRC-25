package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.littletonUtils.AllianceRotateUtil;

import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  private SetpointGenerator() {}

  private static List<Pose2d> m_rightScoring =
      List.of(
          AllianceRotateUtil.apply(new Pose2d(4.95, 2.9, Rotation2d.fromDegrees(120))),
          AllianceRotateUtil.apply(new Pose2d(3.7, 3, Rotation2d.fromDegrees(60))),
          AllianceRotateUtil.apply(new Pose2d(3.25, 4.2, Rotation2d.fromDegrees(0))),
          AllianceRotateUtil.apply(new Pose2d(3.95, 5.2, Rotation2d.fromDegrees(-60))),
          AllianceRotateUtil.apply(new Pose2d(5.2, 5, Rotation2d.fromDegrees(-120))),
          AllianceRotateUtil.apply(new Pose2d(5.75, 3.8, Rotation2d.fromDegrees(180))));

  public static Pose2d getRightScore(Pose2d pose) {
    Logger.recordOutput("nearest", pose.nearest(m_rightScoring));
    return pose.nearest(m_rightScoring);
  }
}
