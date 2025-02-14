package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimbIOReplay implements ClimbIO {

  @Override
  public void updateInputs(ClimbInputs inputs) {}

  @Override
  public void setPID(int slot, double kP, double kI, double kD) {}

  @Override
  public void setSlot(int slot) {}

  @Override
  public void setDesiredAngle(Rotation2d angle, double feedforward) {}

  @Override
  public void zeroEncoder() {}
}
