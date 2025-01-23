package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerInputs {
    public double velocityRPS;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(IntakeRollerInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);
}
