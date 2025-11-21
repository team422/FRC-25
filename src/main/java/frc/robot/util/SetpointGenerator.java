package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.littletonUtils.AllianceFlipUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SetpointGenerator {
  private SetpointGenerator() {}

  // this is actually right for red but left for blue
  // i'll fix it all later but for rn thats just so much work bruh
  // how I will fix so i'm not looked at like a complete bum: change getRightScore to
  // getScoringPosition that takes in our current pose and also left or right
  // then using whether its blue or red i can use the specific array of poses requested
  // might just make four lists to make it seem nicer who's to say
  // for now though, Reed quiz calls
  private static List<Pose2d> m_rightScoring =
      List.of(
          AllianceFlipUtil.apply(new Pose2d(4.95, 2.9, Rotation2d.fromDegrees(120))),
          AllianceFlipUtil.apply(new Pose2d(3.7, 3, Rotation2d.fromDegrees(60))),
          AllianceFlipUtil.apply(new Pose2d(3.25, 4.2, Rotation2d.fromDegrees(0))),
          AllianceFlipUtil.apply(new Pose2d(3.95, 5.2, Rotation2d.fromDegrees(-60))),
          AllianceFlipUtil.apply(new Pose2d(5.2, 5, Rotation2d.fromDegrees(-120))),
          AllianceFlipUtil.apply(new Pose2d(5.75, 3.8, Rotation2d.fromDegrees(180))));

  public static Pose2d getRightScore(Pose2d pose) {
    Logger.recordOutput("nearest", pose.nearest(m_rightScoring));
    return pose.nearest(m_rightScoring);
  }
}
