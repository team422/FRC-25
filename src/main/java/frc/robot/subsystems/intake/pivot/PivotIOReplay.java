package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class PivotIOReplay implements PivotIO {
  @Override
  public void updateInputs(PivotInputs inputs) {}

  @Override
  public void setPIDFF(int slot, double kP, double kI, double kD, double kS) {}

  @Override
  public void setDesiredAngle(Rotation2d angle, double feedforward) {}

  @Override
  public Rotation2d getCurrAngle() {
    return new Rotation2d();
  }

  @Override
  public boolean atSetpoint() {
    return false;
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public void setSlot(int slot) {}

  @Override
  public void zeroEncoder(Angle value) {}
}
