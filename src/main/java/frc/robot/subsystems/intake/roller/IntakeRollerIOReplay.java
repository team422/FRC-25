package frc.robot.subsystems.intake.roller;

public class IntakeRollerIOReplay implements IntakeRollerIO {
  @Override
  public void updateInputs(IntakeRollerInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public boolean hasGamePiece() {
    return false;
  }
}
