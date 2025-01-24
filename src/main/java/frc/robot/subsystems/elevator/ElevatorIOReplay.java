package frc.robot.subsystems.elevator;

public class ElevatorIOReplay implements ElevatorIO {

  @Override
  public void updateInputs(ElevatorInputsAutoLogged inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setDesiredHeight(double meters) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDesiredHeight'");
  }

  @Override
  public void setPIDFF(
      int slot, double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPIDFF'");
  }

  @Override
  public void setSlot(int slot) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setSlot'");
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimits'");
  }

  @Override
  public void setMagic(double velocity, double acceleration, double jerk) {}
}
