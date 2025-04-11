package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOReplay implements PivotIO {
  @Override
  public void updateInputs(PivotInputs inputs) {}

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS) {}

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
}
