package frc.robot.subsystems.manipulator.roller;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class ManipulatorRollerIOReplay implements ManipulatorRollerIO {
  @Override
  public void updateInputs(ManipulatorRollerInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setDesiredPosition(Angle position) {}

  @Override
  public Angle getPosition() {
    return Degrees.zero();
  }

  @Override
  public void setPositionPID(double kP, double kI, double kD, double kS) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public boolean withinPositionTolerance() {
    return false;
  }

  @Override
  public double getCurrent() {
    return 0.0;
  }

  @Override
  public double getAcceleration() {
    return 0.0;
  }
}
