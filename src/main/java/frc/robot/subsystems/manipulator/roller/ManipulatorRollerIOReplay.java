package frc.robot.subsystems.manipulator.roller;

import edu.wpi.first.units.measure.Angle;

public class ManipulatorRollerIOReplay implements ManipulatorRollerIO {
  @Override
  public void updateInputs(ManipulatorRollerInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setDesiredPosition(Angle position) {}

  @Override
  public void setPositionPID(double kP, double kI, double kD, double kS) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}
}
