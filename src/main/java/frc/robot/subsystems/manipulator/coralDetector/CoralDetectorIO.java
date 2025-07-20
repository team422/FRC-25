package frc.robot.subsystems.manipulator.coralDetector;

import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectorIO {
  @AutoLog
  public static class CoralDetectorInputs {
    public boolean manipulatorSensorOne;
    public boolean manipulatorSensorTwo;
    public boolean funnelSensorOne;
    public boolean funnelSensorTwo;
    public boolean hasGamePiece;
    public boolean gamePieceInFunnel;
  }

  public void updateInputs(CoralDetectorInputs inputs);
}
