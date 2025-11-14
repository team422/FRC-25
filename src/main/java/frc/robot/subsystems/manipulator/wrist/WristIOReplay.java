package frc.robot.subsystems.manipulator.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOReplay implements WristIO {

  @Override
  public void updateInputs(WristInputs inputs) {}

  @Override
  public void setAngle(Rotation2d angle) {}

  @Override
  public void setPIDFF(int slot, double p, double i, double d, double kS) {}
}
