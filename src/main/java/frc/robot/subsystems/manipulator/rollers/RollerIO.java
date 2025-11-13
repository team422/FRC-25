package frc.robot.subsystems.manipulator.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs {
    public double voltage;
    public double velocity;
    public boolean connected;
    public double supplyCurrent;
    public double statorCurrent;
    public double temperature;
  }

  public void setVoltage(double volts);

  public void updateInputs(RollerInputs inputs);
}
