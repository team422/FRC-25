package frc.robot.subsystems.elevator;

public class ElevatorIOReplay implements ElevatorIO {

  @Override
  public void updateInputs(ElevatorInputs inputs) {}

  @Override
  public void setDesiredHeight(double meters) {}

  @Override
  public void setPIDFF(
      int slot, double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  @Override
  public void setSlot(int slot) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public void setMagic(double velocity, double acceleration, double jerk) {}

  @Override
  public boolean atSetpoint() {
    return false;
  }

  @Override
  public double getCurrHeight() {
    return 0.0;
  }
}
