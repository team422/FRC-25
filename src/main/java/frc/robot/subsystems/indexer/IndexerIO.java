package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double position;
    public double velocityRPS;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);
}
