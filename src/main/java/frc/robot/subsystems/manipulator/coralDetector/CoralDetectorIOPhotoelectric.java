package frc.robot.subsystems.manipulator.coralDetector;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralDetectorIOPhotoelectric implements CoralDetectorIO {
  private DigitalInput m_photoElectricOne;
  private DigitalInput m_photoElectricTwo;

  public CoralDetectorIOPhotoelectric(int photoElectricOnePort, int photoElectricTwoPort) {
    m_photoElectricOne = new DigitalInput(photoElectricOnePort);
    m_photoElectricTwo = new DigitalInput(photoElectricTwoPort);
  }

  @Override
  public void updateInputs(CoralDetectorInputs inputs) {
    inputs.sensorOne = photoElectricOneDetected();
    inputs.sensorTwo = photoElectricTwoDetected();
  }

  @Override
  public boolean hasGamePiece() {
    return photoElectricOneDetected() && photoElectricTwoDetected();
  }

  // photoelectrics return false when detected so we invert
  private boolean photoElectricOneDetected() {
    return m_photoElectricOne.get();
  }

  private boolean photoElectricTwoDetected() {
    return m_photoElectricTwo.get();
  }
}
