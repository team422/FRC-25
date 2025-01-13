package frc.robot.subsystems.aprilTagVision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  public static class AprilTagVisionInputs implements LoggableInputs {
    public double[] timestamps;
    public double[][] frames;
    public double[] demoFrame;
    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", frames.length);
      // akit doesnt support 2d array so gotta do it manually
      for (int i = 0; i < frames.length; i++) {
        table.put("Frame/" + i, frames[i]);
      }
      table.put("DemoFrame", demoFrame);
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", new double[] {0.0});
      int frameCount = table.get("FrameCount", 0);
      frames = new double[frameCount][];
      for (int i = 0; i < frameCount; i++) {
        frames[i] = table.get("Frame/" + i, new double[] {});
      }
      demoFrame = table.get("DemoFrame", new double[] {});
      fps = table.get("Fps", 0);
    }
  }

  public void updateInputs(AprilTagVisionInputs inputs);
}
