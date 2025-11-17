package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeInputs {
    public double topVoltage;
    public double sideVoltage;
    public double topVelocity;
    public double sideVelocity;
    public boolean topConnected;
    public boolean sideConnected;
    public double topSupplyCurrent;
    public double sideSupplyCurrent;
    public double topStatorCurrent;
    public double sideStatorCurrent;
    public double topTemperature;
    public double bottomTemperature;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double top, double side);
}
