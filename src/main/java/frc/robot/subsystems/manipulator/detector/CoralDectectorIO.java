package frc.robot.subsystems.manipulator.detector;

import org.littletonrobotics.junction.AutoLog;

public interface CoralDectectorIO {
  @AutoLog
  public class DetectorInputs {
    public boolean manipSensorOne;
    public boolean manipSensorTwo;
    public boolean funnelSensorOne;
    public boolean funnelSensorTwo;
    public boolean hasGamePiece;
    public boolean gamePieceInFunnel;
  }

  public void updateInputs(DetectorInputs inputs);
}
