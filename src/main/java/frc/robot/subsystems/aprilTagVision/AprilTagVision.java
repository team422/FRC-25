package frc.robot.subsystems.aprilTagVision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.EqualsUtil;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagVisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIO.AprilTagVisionInputs;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  public record VisionObservation(
      Pose2d visionPose, double timestamp, Matrix<N3, N1> standardDeviations) {}

  private AprilTagVisionIO[] m_ios;
  private AprilTagVisionInputs[] m_inputs;

  private Map<Integer, Double> m_lastFrameTimes;
  private Map<Integer, Double> m_lastTagDetectionTimes;

  private Alert[] m_noReadingsAlerts;

  private boolean m_firstMeasurement = true;

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
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    for (int i = 0; i < m_ios.length; i++) {
      m_ios[i].updateInputs(m_inputs[i]);

      Logger.processInputs("AprilTagVision/Inst" + i, m_inputs[i]);
    }

    List<Pose3d> allCameraPoses = new ArrayList<>();
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < m_ios.length; instanceIndex++) {
      for (int frameIndex = 0;
          frameIndex < m_inputs[instanceIndex].timestamps.length;
          frameIndex++) {
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
        double error = 0;
        switch (tagCount) {
          case 0:
            // No tag, do nothing
            continue;
          case 1:
            // One possible pose (multiple tags detected), can use directly
            double translationX = values[2];
            double translationY = values[3];
            double translationZ = values[4];
            double rotationW = values[5];
            double rotationX = values[6];
            double rotationY = values[7];
            double rotationZ = values[8];
            cameraPose =
                new Pose3d(
                    new Translation3d(translationX, translationY, translationZ),
                    new Rotation3d(new Quaternion(rotationW, rotationX, rotationY, rotationZ)));

            robotPose3d =
                cameraPose.transformBy(
                    AprilTagVisionConstants.kCameraTransforms[instanceIndex].inverse());

            useVisionRotation = true;

            error = values[1];

            break;
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

            // TODO: add blacklist TO PI, NOT HERE

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
              // since the cameras have some roll, the false reprojection will be up in the air or
              // in the ground
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

        // if camera error low, distance to tag small, robot not moving, we can trust rotation
        // should only ever be single tag but leaving it in for now in case we switch
        if (!useVisionRotation) {
          Pose3d tagPose = tagPoses.get(0);
          double distance = tagPose.getTranslation().getDistance(cameraPose.getTranslation());
          ChassisSpeeds speeds = RobotState.getInstance().getRobotSpeeds();
          if (error < AprilTagVisionConstants.kRotationErrorThreshold
              && distance < AprilTagVisionConstants.kRotationDistanceThreshold
              && speeds.vxMetersPerSecond < AprilTagVisionConstants.kRotationSpeedThreshold
              && speeds.vyMetersPerSecond < AprilTagVisionConstants.kRotationSpeedThreshold) {
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
        xyStandardDeviation =
            AprilTagVisionConstants.kXYStandardDeviationCoefficient.get()
                // evil math
                // if the error is under the threshold, we make the standard deviation smaller
                // but if the error is above the threshold, we make the standard deviation larger
                // i had to use desmos for this
                // may be added back later but this was causing me some issues
                // * Math.pow(error + 1 - AprilTagVisionConstants.kErrorStandardDeviationThreshold,
                // 4)

                // back to normal math
                * Math.pow(averageDistance, 2.0)
                / tagPoses.size();

        double thetaStandardDeviation = 1;
        if (RobotState.getInstance().getNumVisionGyroObservations() < 100) {
          // if we don't have many gyro observations, we need theta measurements
          useVisionRotation = true;
        }

        if (useVisionRotation) {
          thetaStandardDeviation =
              AprilTagVisionConstants.kThetaStandardDeviationCoefficient.get()
                  * Math.pow(averageDistance, 2.0)
                  / tagPoses.size();
        } else {
          thetaStandardDeviation = Double.POSITIVE_INFINITY;
        }

        double gyroAccuracyFactor = 1.0;
        if (RobotState.getInstance().getNumVisionGyroObservations() > 100
            && !DriverStation.isDisabled()) {
          // Calculate difference between vision and gyro rotation
          Rotation2d visionRotation = robotPose.getRotation();
          Rotation2d gyroRotation = RobotState.getInstance().getRobotPose().getRotation();
          double angleDifference = Math.abs(visionRotation.minus(gyroRotation).getDegrees());
          // if we are more than 1 degree off, reduce accuracy
          // now we don't just wanna 100% trust it if it has the same rotation
          gyroAccuracyFactor =
              Math.max(0.3, angleDifference * AprilTagVisionConstants.kGyroAccurary);
          // if it is 2 degrees off, we are increasing our standard deviation by 2
          xyStandardDeviation *= gyroAccuracyFactor;
          // this should not change our theta standard deviation as our gyroscope will not correct
          // if we do
        } else {
          // if we don't have many gyro observations, we should trust theta more
          thetaStandardDeviation *= 0.01;
        }
        if (thetaStandardDeviation
            < 0.1) // check if the rotation standard deviation is low, if so add to
        // numVisionGyroObservations
        {
          RobotState.getInstance().incrementNumVisionGyroObservations();
        }

        if (DriverStation.isDisabled()) {
          xyStandardDeviation *= 0.1;
        }

        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/Transform",
            AprilTagVisionConstants.kCameraTransforms[instanceIndex]);

        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/StandardDeviationXY", xyStandardDeviation);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/StandardDeviationTheta",
            thetaStandardDeviation);

        allVisionObservations.add(
            new VisionObservation(
                robotPose,
                timestamp,
                VecBuilder.fill(xyStandardDeviation, xyStandardDeviation, thetaStandardDeviation)));

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

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> entry : m_lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - entry.getValue()
          < AprilTagVisionConstants.kTargetLogTimeSecs) {
        allTagPoses.add(AprilTagVisionConstants.kAprilTagLayout.getTagPose(entry.getKey()).get());
      }
    }

    // Send to RobotState
    // TODO: mess around
    int maxObservations = 10;
    if (allVisionObservations.size() > maxObservations) {
      allVisionObservations = allVisionObservations.subList(0, maxObservations);
    }

    if (m_firstMeasurement) {
      for (int i = 0; i < allVisionObservations.size(); i++) {
        var curr = allVisionObservations.get(i);
        curr = new VisionObservation(curr.visionPose, curr.timestamp, VecBuilder.fill(0, 0, 0));
        allVisionObservations.set(i, curr);
      }
      m_firstMeasurement = false;
    }

    if (EqualsUtil.epsilonEquals(AprilTagVisionConstants.kUseVision.get(), 1.0)) {
      allVisionObservations.stream()
          .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
          .forEach(RobotState.getInstance()::addVisionObservation);
    }

    if (Constants.kUseAlerts) {
      for (int i = 0; i < m_noReadingsAlerts.length; i++) {
        double lastTime = m_lastFrameTimes.get(i);
        double elapsed = Timer.getFPGATimestamp() - lastTime;
        if (elapsed > AprilTagVisionConstants.kDisconnectTimeout) {
          m_noReadingsAlerts[i].set(true);
          RobotState.getInstance().triggerAlert();
        }
      }
    }

    Logger.recordOutput("PeriodicTime/AprilTagVision", (HALUtil.getFPGATime() - start) / 1000.0);
  }
}
