package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double sidePosition;
    public double sideVelocityRPS;
    public double sideCurrent;
    public double sideStatorCurrent;
    public double sideVoltage;
    public double sideTemperature;
    public boolean sideMotorIsConnected;
    public double topPosition;
    public double topVelocityRPS;
    public double topCurrent;
    public double topStatorCurrent;
    public double topVoltage;
    public double topTemperature;
    public boolean topMotorIsConnected;
    public boolean sideMotorDataUpdate;
  }

  public void updateInputs(IndexerInputs inputs);

  public void setVoltage(double sideVoltage, double topVoltage);

  public void setCurrentLimits(double supplyLimit);
}
