package frc.robot.subsystems.manipulator.coralDetector;

public class CoralDetectorIOReplay implements CoralDetectorIO {
  @Override
  public void updateInputs(CoralDetectorInputs inputs) {}

  @Override
  public boolean hasGamePiece() {
    return false;
  }

  public boolean gamePieceInFunnel() {
    return false;
  }
}
