package frc.robot.subsystems.manipulator.coralDetector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralDetectorIOPhotoelectric implements CoralDetectorIO {
  private DigitalInput m_manipulatorSensorOne;
  private DigitalInput m_manipulatorSensorTwo;
  private DigitalInput m_funnelSensorOne;
  private DigitalInput m_funnelSensorTwo;

  // sometimes the funnel photoelectrics false trigger so we use a rising debounce
  // we don't want to have the debounce falling so that our indexing is not delayed
  private Debouncer m_funnelSensorOneDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private Debouncer m_funnelSensorTwoDebouncer = new Debouncer(0.1, DebounceType.kRising);

  public CoralDetectorIOPhotoelectric(
      int manipulatorSensorOnePort,
      int manipulatorSensorTwoPort,
      int funnelSensorOnePort,
      int funnelSensorTwoPort) {
    m_manipulatorSensorOne = new DigitalInput(manipulatorSensorOnePort);
    m_manipulatorSensorTwo = new DigitalInput(manipulatorSensorTwoPort);
    m_funnelSensorOne = new DigitalInput(funnelSensorOnePort);
    m_funnelSensorTwo = new DigitalInput(funnelSensorTwoPort);
  }

  @Override
  public void updateInputs(CoralDetectorInputs inputs) {
    inputs.manipulatorSensorOne = photoElectricManiOneDetected();
    inputs.manipulatorSensorTwo = photoElectricManiTwoDetected();
    inputs.funnelSensorOne = photoElectricFunnelOneDetected();
    inputs.funnelSensorTwo = photoElectricFunnelTwoDetected();
  }

  @Override
  public boolean hasGamePiece() {
    return photoElectricManiOneDetected() && photoElectricManiTwoDetected();
  }

  public boolean gamePieceInFunnel() {
    return photoElectricFunnelOneDetected() || photoElectricFunnelTwoDetected();
  }

  private boolean photoElectricManiOneDetected() {
    return m_manipulatorSensorOne.get();
  }

  private boolean photoElectricManiTwoDetected() {
    return m_manipulatorSensorTwo.get();
  }

  // these photoelectrics give different readings from the manipulator ones
  private boolean photoElectricFunnelOneDetected() {
    return m_funnelSensorOneDebouncer.calculate(!m_funnelSensorOne.get());
  }

  private boolean photoElectricFunnelTwoDetected() {
    return m_funnelSensorTwoDebouncer.calculate(!m_funnelSensorTwo.get());
  }
}
