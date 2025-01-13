package frc.robot.subsystems.aprilTagVision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
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
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          AprilTagVisionConstants.kCameraTransforms[AprilTagVisionConstants.kCalibIndex] =
              new Transform3d(
                  new Translation3d(
                      AprilTagVisionConstants.transformCameraX.get(),
                      AprilTagVisionConstants.cameraY.get(),
                      AprilTagVisionConstants.cameraZ.get()),
                  new Rotation3d(
                      AprilTagVisionConstants.cameraRoll.get(),
                      AprilTagVisionConstants.cameraPitch.get(),
                      AprilTagVisionConstants.cameraYaw.get()));
        },
        AprilTagVisionConstants.transformCameraX,
        AprilTagVisionConstants.cameraY,
        AprilTagVisionConstants.cameraZ,
        AprilTagVisionConstants.cameraRoll,
        AprilTagVisionConstants.cameraPitch,
        AprilTagVisionConstants.cameraYaw);

    double start = Timer.getFPGATimestamp();

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
              // Select based on odometry estimated rotation (which projection is closer)
              Rotation2d currentRotation =
                  RobotState.getInstance().getRobotPose().getRotation();
              Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
              Rotation2d visionRotation2 = robotPose3d2.toPose2d().getRotation();
              double angle1 = Math.abs(currentRotation.minus(visionRotation1).getRadians());
              double angle2 = Math.abs(currentRotation.minus(visionRotation2).getRadians());
              if (angle1 < angle2) {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
              } else {
                cameraPose = cameraPose2;
                robotPose3d = robotPose3d2;
              }
            }

            useVisionRotation =
                false; // since there are two possible poses we shouldn't rely on the rotation

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

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double averageDistance = totalDistance / tagPoses.size();

        // Add observation to list
        double xyStandardDeviation = 1;
        if (edu.wpi.first.wpilibj.RobotState.isAutonomous()) {
          xyStandardDeviation =
              3.3
                  * AprilTagVisionConstants.kXYStandardDeviationCoefficient.get()
                  * Math.pow(averageDistance, 2.0)
                  / tagPoses.size();
        } else {
          xyStandardDeviation =
              AprilTagVisionConstants.kXYStandardDeviationCoefficient.get()
                  * Math.pow(averageDistance, 2.0)
                  / tagPoses.size();
        }
        double thetaStandardDeviation = 1;
        if (useVisionRotation) {
          thetaStandardDeviation =
              AprilTagVisionConstants.kThetaStandardDeviationCoefficient.get()
                  * Math.pow(averageDistance, 2.0)
                  / tagPoses.size();
        } else {
          thetaStandardDeviation = Double.POSITIVE_INFINITY;
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
    int maxObservations = 10;
    if (allVisionObservations.size() > maxObservations) {
      allVisionObservations = allVisionObservations.subList(0, maxObservations);
    }

    allVisionObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);

    Logger.recordOutput("PeriodicTime/AprilTagVision", Timer.getFPGATimestamp() - start);
  }
}
