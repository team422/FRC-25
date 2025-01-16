package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelInputs {
    public double currVelocityRPS;
    public double desiredVelocityRPS;
    public boolean atSetpoint;
    public boolean velocityControl;
    public double voltage;
    public double current;
  }

  public void updateInputs(FlywheelInputs inputs);

  public void setPIDFF(double kP, double kI, double kD, double kS, double kV);

  public void setDesiredVelocity(double velocityRPS);

  public void setVoltage(double voltage);
}
