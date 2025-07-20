// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.aprilTagVision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AprilTagVisionConstants;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 7;
  private static final int cameraGain = 20;
  private static final int cameraBrightness = 20;
  private static final int cameraContrast = 64;
  private static final int cameraGamma = 85;
  // all non-ground tags
  private static final long[] tagIDBlacklist = new long[] {1, 2, 3, 4, 5, 12, 13, 14, 15, 16};

  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber demoObservationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  private final IntegerPublisher enabledPublisher;
  private int enabledValue = 0;

  public AprilTagVisionIONorthstar(String instanceId, String cameraId) {
    var northstarTable = NetworkTableInstance.getDefault().getTable(instanceId);

    var configTable = northstarTable.getSubTable("config");
    configTable.getStringTopic("camera_id").publish().set(cameraId);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
    configTable.getIntegerTopic("camera_brightness").publish().set(cameraBrightness);
    configTable.getIntegerTopic("camera_contrast").publish().set(cameraContrast);
    configTable.getIntegerTopic("camera_gamma").publish().set(cameraGamma);
    configTable
        .getDoubleTopic("fiducial_size_m")
        .publish()
        .set(AprilTagVisionConstants.kAprilTagWidth);
    configTable.getIntegerArrayTopic("tag_id_blacklist").publish().set(tagIDBlacklist);

    enabledPublisher = configTable.getIntegerTopic("robot_enabled").publish();
    enabledPublisher.set(0);

    try {
      configTable
          .getStringTopic("tag_layout")
          .publish()
          .set(new ObjectMapper().writeValueAsString(AprilTagVisionConstants.kAprilTagLayout));
    } catch (JsonProcessingException e) {
      throw new RuntimeException("Failed to serialize AprilTag layout JSON for Northstar");
    }

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    demoObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("demo_observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

    var genericNorthstarTable = NetworkTableInstance.getDefault().getTable("northstar");
    var commandsTable = genericNorthstarTable.getSubTable("commands");
    commandsTable.getIntegerTopic("kys").publish().set(0);
  }

  public void updateInputs(AprilTagVisionInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
      inputs.frames[i] = queue[i].value;
    }
    inputs.demoFrame = new double[] {};
    for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
      inputs.demoFrame = demoFrame;
    }
    inputs.fps = fpsSubscriber.get();

    if (enabledValue != (DriverStation.isEnabled() ? 1 : 0)) {
      enabledPublisher.set(DriverStation.isEnabled() ? 1 : 0);
    }
  }
}
