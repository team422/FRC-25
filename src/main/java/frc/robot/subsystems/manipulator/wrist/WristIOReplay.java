package frc.robot.subsystems.manipulator.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOReplay implements WristIO {
  @Override
  public void updateInputs(WristInputs inputs) {}

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS) {}

  @Override
  public void setDesiredAngle(Rotation2d angle) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}
}
