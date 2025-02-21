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
    return Degrees.of(0);
  }

  @Override
  public void setPositionPID(double kP, double kI, double kD) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}
}
