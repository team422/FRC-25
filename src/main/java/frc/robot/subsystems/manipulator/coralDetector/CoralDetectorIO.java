package frc.robot.subsystems.manipulator.coralDetector;

import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectorIO {
  @AutoLog
  public static class CoralDetectorInputs {
    public boolean sensorOne;
    public boolean sensorTwo;
  }

  public void updateInputs(CoralDetectorInputs inputs);

  public boolean hasGamePiece();
}
