package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs {
    public double voltage;
    public double current;
    public double output;
  }

  public void updateInputs(RollerInputs inputs);

  public void setVoltage(double voltage);
}
