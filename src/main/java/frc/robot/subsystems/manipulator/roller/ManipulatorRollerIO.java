package frc.robot.subsystems.manipulator.roller;

import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorRollerIO {
  @AutoLog
  public static class ManipulatorRollerInputs {
    public double velocityRPS;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(ManipulatorRollerInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);
}
