package frc.robot.subsystems.elevator;

public class ElevatorIOReplay implements ElevatorIO {

  @Override
  public void updateInputs(ElevatorInputs inputs) {}

  @Override
  public void setDesiredHeight(double meters) {}

  @Override
  public void setPIDFF(int slot, double kP, double kI, double kD, double kG) {}

  @Override
  public void setSlot(int slot) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public void zeroElevator() {}

  @Override
  public void setVoltage(double voltage) {}
}
