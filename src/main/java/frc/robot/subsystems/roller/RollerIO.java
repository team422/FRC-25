package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerInputs {
    public double voltage;
    public double velocity;
    public double current;
  }

  public void updateInputs(RollerInputs inputs);

  public void setVoltage(double voltage);
}
