package frc.robot.subsystems.flywheel;

public class FlywheelIOReplay implements FlywheelIO {
  @Override
  public void updateInputs(FlywheelInputs inputs) {}

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kV) {}

  @Override
  public void setDesiredVelocity(double velocityRPS) {}

  @Override
  public void setVoltage(double voltage) {}
}
