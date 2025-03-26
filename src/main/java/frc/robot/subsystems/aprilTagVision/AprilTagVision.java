package frc.robot.subsystems.aprilTagVision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagVisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIO.AprilTagVisionInputs;
import frc.robot.util.AlertManager;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private AprilTagVisionIO[] m_ios;
  private AprilTagVisionInputs[] m_inputs;

  private Map<Integer, Double> m_lastFrameTimes;
  private Map<Integer, Double> m_lastTagDetectionTimes;

  private Alert[] m_noReadingsAlerts;

  private boolean m_firstMeasurement = true;

  private int m_autoAutoScoreMeasurements = 0;

  private Timer m_alertDeadzoneTimer = new Timer();

  public AprilTagVision(AprilTagVisionIO... ios) {
    m_ios = ios;
    m_inputs = new AprilTagVisionInputs[m_ios.length];
    for (int i = 0; i < m_ios.length; i++) {
      m_inputs[i] = new AprilTagVisionInputs();
    }

    // Create map of last frame times for instances
    m_lastFrameTimes = new HashMap<>();
    for (int i = 0; i < m_ios.length; i++) {
      m_lastFrameTimes.put(i, 0.0);
    }

    // Create map of last detection times for tags
    m_lastTagDetectionTimes = new HashMap<>();
    AprilTagVisionConstants.kAprilTagLayout
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              m_lastTagDetectionTimes.put(tag.ID, 0.0);
            });

    m_noReadingsAlerts = new Alert[m_ios.length];
    for (int i = 0; i < m_noReadingsAlerts.length; i++) {
      m_noReadingsAlerts[i] =
          new Alert(String.format("April Tag Vision %d: No Readings", i), AlertType.kError);
    }

    m_alertDeadzoneTimer.start();

    AlertManager.registerAlert(m_noReadingsAlerts);
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    for (int i = 0; i < m_ios.length; i++) {
      m_ios[i].updateInputs(m_inputs[i]);

      Logger.processInputs("AprilTagVision/Inst" + i, m_inputs[i]);
    }

    // if (RobotState.getInstance().getYawVelocity() > 1.0) {
    //   Logger.recordOutput("AprilTagVision/TooFast", true);
    //   return;
    // }
    // Logger.recordOutput("AprilTagVision/TooFast", false);

    List<Pose3d> allCameraPoses = new ArrayList<>();
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<TimestampedVisionUpdate> allVisionUpdates = new ArrayList<>();
    List<Pose3d> allSkippedPoses = new ArrayList<>();
    List<Pose3d> allGod = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < m_ios.length; instanceIndex++) {
      for (int frameIndex = 0;
          frameIndex < m_inputs[instanceIndex].timestamps.length && allRobotPoses.size() < 10;
          frameIndex++) {
        if (m_inputs[instanceIndex].timestamps[frameIndex] == 0) {
          continue;
        }
        m_lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        double timestamp = m_inputs[instanceIndex].timestamps[frameIndex];

        if (m_inputs[instanceIndex].fps > 0) {
          timestamp -= 1 / m_inputs[instanceIndex].fps;
        }

        // values from northstar
        double[] values = m_inputs[instanceIndex].frames[frameIndex];

        if (values.length == 0) {
          continue;
        }

        int tagCount = (int) values[0];

        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;

        @SuppressWarnings("unused")
        double error = 0;

        switch (tagCount) {
          case 0:
            // No tag, do nothing
            continue;
          case 1:
            continue;
            // One possible pose (multiple tags detected), can use directly
            // double translationX = values[2];
            // double translationY = values[3];
            // double translationZ = values[4];
            // double rotationW = values[5];
            // double rotationX = values[6];
            // double rotationY = values[7];
            // double rotationZ = values[8];
            // cameraPose =
            //     new Pose3d(
            //         new Translation3d(translationX, translationY, translationZ),
            //         new Rotation3d(new Quaternion(rotationW, rotationX, rotationY, rotationZ)));

            // robotPose3d =
            //     cameraPose.transformBy(
            //         AprilTagVisionConstants.kCameraTransforms[instanceIndex].inverse());

            // useVisionRotation = true;

            // error = values[1];

            // break;
          case 2:
            // Two possible poses (one tag detected), need to choose one

            // lotta variables here but i wanna make it clear
            // First pose
            double translationX1 = values[2];
            double translationY1 = values[3];
            double translationZ1 = values[4];
            double rotationW1 = values[5];
            double rotationX1 = values[6];
            double rotationY1 = values[7];
            double rotationZ1 = values[8];

            // Second pose
            double translationX2 = values[10];
            double translationY2 = values[11];
            double translationZ2 = values[12];
            double rotationW2 = values[13];
            double rotationX2 = values[14];
            double rotationY2 = values[15];
            double rotationZ2 = values[16];

            Pose3d cameraPose1 =
                new Pose3d(
                    new Translation3d(translationX1, translationY1, translationZ1),
                    new Rotation3d(new Quaternion(rotationW1, rotationX1, rotationY1, rotationZ1)));

            Pose3d cameraPose2 =
                new Pose3d(
                    new Translation3d(translationX2, translationY2, translationZ2),
                    new Rotation3d(new Quaternion(rotationW2, rotationX2, rotationY2, rotationZ2)));

            Pose3d robotPose3d1 =
                cameraPose1.transformBy(
                    AprilTagVisionConstants.kCameraTransforms[instanceIndex].inverse());
            Pose3d robotPose3d2 =
                cameraPose2.transformBy(
                    AprilTagVisionConstants.kCameraTransforms[instanceIndex].inverse());

            double error1 = values[1];
            double error2 = values[9];

            // Check for ambiguity
            if (error1 < error2 * AprilTagVisionConstants.kAmbiguityThreshold
                || error2 < error1 * AprilTagVisionConstants.kAmbiguityThreshold) {
              // since the cameras have some pitch, the false reprojection will be in the air
              // so we can just choose the one that is closer to the floor
              double height1 = Math.abs(robotPose3d1.getZ());
              double height2 = Math.abs(robotPose3d2.getZ());
              // double pitch1 = Math.abs(robotPose3d1.getRotation().getY());
              // double pitch2 = Math.abs(robotPose3d2.getRotation().getY());
              // double roll1 = Math.abs(robotPose3d1.getRotation().getX());
              // double roll2 = Math.abs(robotPose3d2.getRotation().getX());
              // double yaw1 = Math.abs(robotPose3d1.getRotation().getZ());
              // double yaw2 = Math.abs(robotPose3d2.getRotation().getZ());
              if (height1 < height2) {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
                error = error1;
              } else {
                cameraPose = cameraPose2;
                robotPose3d = robotPose3d2;
                error = error2;
              }
            }

            Logger.recordOutput(
                "AprilTagVision/Inst" + instanceIndex + "/robotPose3d1", robotPose3d1);
            Logger.recordOutput(
                "AprilTagVision/Inst" + instanceIndex + "/robotPose3d2", robotPose3d2);
            Logger.recordOutput(
                "AprilTagVision/Inst" + instanceIndex + "/cameraPose1", cameraPose1);
            Logger.recordOutput(
                "AprilTagVision/Inst" + instanceIndex + "/cameraPose2", cameraPose2);

            break;
        }

        // Exit if no valid pose
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off field
        if (robotPose3d.getX() < -AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getX()
                > FieldConstants.kFieldLength + AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getY() < -AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getY()
                > FieldConstants.kFieldWidth + AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getZ() < -AprilTagVisionConstants.kZMargin
            || robotPose3d.getZ() > AprilTagVisionConstants.kZMargin) {
          continue;
        }

        // Exit if the pitch and roll are cooked (combined >20 degrees)
        double pitch = Units.radiansToDegrees(Math.abs(robotPose3d.getRotation().getY()));
        double roll = Units.radiansToDegrees(Math.abs(robotPose3d.getRotation().getX()));
        if (pitch + roll > 20) {
          continue;
        }

        // Get 2D Pose
        Pose2d robotPose = robotPose3d.toPose2d();

        double elapsedMicroseconds;
        // Depending on the tag count the elapsed time is at a different index
        if (tagCount == 1) {
          elapsedMicroseconds = values[9];
        } else {
          elapsedMicroseconds = values[17];
        }
        double elapsedSeconds = elapsedMicroseconds / 1e6;
        timestamp -= elapsedSeconds;

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        int startIndex;
        // Depending on the tag count the poses start at different indices
        if (tagCount == 1) {
          startIndex = 10;
        } else {
          startIndex = 18;
        }
        for (int i = startIndex; i < values.length; i++) {
          int tagId = (int) values[i];
          m_lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose = AprilTagVisionConstants.kAprilTagLayout.getTagPose(tagId);
          tagPose.ifPresent(tagPoses::add);
        }
        Pose3d tagPose0 = tagPoses.get(0);
        double distance = tagPose0.getTranslation().getDistance(cameraPose.getTranslation());
        Logger.recordOutput(
            "AprilTagVision/distance",
            Math.abs(
                RobotState.getInstance()
                    .getRobotPose()
                    .getRotation()
                    .minus(robotPose.getRotation())
                    .getDegrees()));
        if (Math.abs(
                    RobotState.getInstance()
                        .getRobotPose()
                        .getRotation()
                        .minus(robotPose.getRotation())
                        .getDegrees())
                > 5
            && distance > Units.inchesToMeters(50)) {
          allSkippedPoses.add(robotPose3d);
          continue;
        }

        double yawDifference =
            Math.abs(
                RobotState.getInstance()
                    .getRobotPose()
                    .getRotation()
                    .minus(robotPose.getRotation())
                    .getDegrees());

        if (yawDifference < 0.5) {
          allGod.add(robotPose3d);
        }

        // if camera error low, distance to tag small, robot not moving, we can trust rotation
        // should only ever be single tag but leaving it in for now in case we switch
        if (!useVisionRotation) {
          // Pose3d tagPose = tagPoses.get(0);
          // double distance = tagPose.getTranslation().getDistance(cameraPose.getTranslation());
          ChassisSpeeds speeds = RobotState.getInstance().getRobotSpeeds();
          if (distance < AprilTagVisionConstants.kRotationDistanceThreshold
              && Math.abs(speeds.vxMetersPerSecond)
                  < AprilTagVisionConstants.kRotationSpeedThreshold
              && Math.abs(speeds.vyMetersPerSecond)
                  < AprilTagVisionConstants.kRotationSpeedThreshold) {
            useVisionRotation = true;
          }
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double averageDistance = totalDistance / tagPoses.size();

        // Add observation to list
        double xyStandardDeviation = 1;
        if (averageDistance < Units.inchesToMeters(50)) {
          xyStandardDeviation = 0.01;
        } else {
          xyStandardDeviation =
              0.01
                  // back to normal math
                  * averageDistance
                  / tagPoses.size();
        }

        // xyStandardDeviation *= yawDifference * 0.25 + 0.1;

        double thetaStandardDeviation = 1;
        if (RobotState.getInstance().getNumVisionGyroObservations() < 100) {
          // if we don't have many gyro observations, we need theta measurements
          useVisionRotation = true;
        }

        if (useVisionRotation) {
          thetaStandardDeviation = 0.05 * Math.pow(averageDistance, 2.0) / tagPoses.size();
        } else {
          thetaStandardDeviation = Double.POSITIVE_INFINITY;
        }

        // double gyroAccuracyFactor = 1.0;
        // if (RobotState.getInstance().getNumVisionGyroObservations() > 100) {
        //   // Calculate difference between vision and gyro rotation
        //   Rotation2d visionRotation = robotPose.getRotation();
        //   Rotation2d gyroRotation = RobotState.getInstance().getRobotPose().getRotation();
        //   double angleDifference = Math.abs(visionRotation.minus(gyroRotation).getDegrees());
        //   // if we are more than 1 degree off, reduce accuracy
        //   // now we don't just wanna 100% trust it if it has the same rotation
        //   gyroAccuracyFactor =
        //       Math.max(0.3, angleDifference * AprilTagVisionConstants.kGyroAccurary);
        //   // if it is 2 degrees off, we are increasing our standard deviation by 2
        //   xyStandardDeviation *= gyroAccuracyFactor;
        //   // this should not change our theta standard deviation as our gyroscope will not
        // correct
        //   // if we do
        // }
        // else {
        //   // if we don't have many gyro observations, we should trust theta more
        //   thetaStandardDeviation *= 0.01;
        // }
        if (thetaStandardDeviation
            < 0.1) // check if the rotation standard deviation is low, if so add to
        // numVisionGyroObservations
        {
          RobotState.getInstance().incrementNumVisionGyroObservations();
        }

        if (DriverStation.isDisabled()) {
          // xyStandardDeviation *= 0.1;
        }

        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/Transform",
            AprilTagVisionConstants.kCameraTransforms[instanceIndex]);

        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/StandardDeviationXY", xyStandardDeviation);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/StandardDeviationTheta",
            thetaStandardDeviation);

        // if (yawDifference < 0.5) {
        //   allVisionUpdates.add(
        //       new TimestampedVisionUpdate(timestamp, robotPose, VecBuilder.fill(0, 0, 0)));
        // }
        // if (DriverStation.isDisabled()) {

        allVisionUpdates.add(
            new TimestampedVisionUpdate(
                timestamp,
                robotPose,
                VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation)));
        // }
        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);

        // this will work?
        allCameraPoses.add(cameraPose);

        // Log data from instance
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
            Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);

        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));

        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/CameraPose", cameraPose);
      }
    }

    // Log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));
    Logger.recordOutput("AprilTagVision/CameraPoses", allCameraPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("AprilTagVision/SkippedPoses", allSkippedPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("AprilTagVision/God", allGod.toArray(Pose3d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> entry : m_lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - entry.getValue()
          < AprilTagVisionConstants.kTargetLogTimeSecs) {
        allTagPoses.add(AprilTagVisionConstants.kAprilTagLayout.getTagPose(entry.getKey()).get());
      }
    }

    if (m_firstMeasurement && !allVisionUpdates.isEmpty()) {
      // for (int i = 0; i < allVisionUpdates.size(); i++) {
      //   var curr = allVisionUpdates.get(i);
      //   curr = new TimestampedVisionUpdate(curr.timestamp(), curr.pose(), VecBuilder.fill(0, 0,
      // 0));
      //   allVisionUpdates.set(i, curr);
      // }
      Logger.recordOutput("AprilTagVision/First", allVisionUpdates.get(0).pose());
      RobotState.getInstance().setDrivePose(allVisionUpdates.get(0).pose());
      m_firstMeasurement = false;
    }

    // Send to RobotState
    RobotState.getInstance().addTimestampedVisionObservations(allVisionUpdates);

    if (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoAutoScore) {
      m_autoAutoScoreMeasurements += allVisionUpdates.size();
      Logger.recordOutput("AprilTagVision/AutoAutoScoreMeasurements", m_autoAutoScoreMeasurements);
    }

    if (Constants.kUseAlerts && m_alertDeadzoneTimer.hasElapsed(20.0)) {
      for (int i = 0; i < m_noReadingsAlerts.length; i++) {
        double lastTime = m_lastFrameTimes.get(i);
        double elapsed = Timer.getFPGATimestamp() - lastTime;
        if (elapsed > AprilTagVisionConstants.kDisconnectTimeout) {
          m_noReadingsAlerts[i].set(true);
        } else {
          m_noReadingsAlerts[i].set(false);
        }
      }
    }

    Logger.recordOutput("PeriodicTime/AprilTagVision", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public int getAutoAutoScoreMeasurements() {
    return m_autoAutoScoreMeasurements;
  }

  public void resetAutoAutoScoreMeasurements() {
    m_autoAutoScoreMeasurements = 0;
  }
}
