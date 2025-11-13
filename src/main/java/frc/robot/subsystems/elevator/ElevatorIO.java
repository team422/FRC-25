package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public double desiredHeight;
    public double leadingHeight;
    public double followingHeight;
    public boolean atSetpoint;
    public boolean leadingMotorConnected;
    public boolean followingMotorConnected;
    public double leadingVoltage;
    public double followingVoltage;
    public double leadingSupplyCurrent;
    public double followingSupplyCurrent;
    public double leadingStatorCurrent;
    public double followingStatorCurrent;
    public double leadingTemperature;
    public double followingTemperature;
    public double leadingVelocity;
    public double followingVelocity;
  }

  public void updateInputs(ElevatorInputs inputs);

  public void setHeight(double height);

  public void setPIDFF(int slot, double p, double i, double d, double kG);
}
