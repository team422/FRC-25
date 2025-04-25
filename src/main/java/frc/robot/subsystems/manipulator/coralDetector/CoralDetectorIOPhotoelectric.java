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

  // the manipulator photoelectric one gives incorrect false values sometimes so we need falling
  private Debouncer m_manipulatorSensorOneDebouncer = new Debouncer(0.1, DebounceType.kFalling);
  private Debouncer m_manipulatorSensorTwoDebouncer = new Debouncer(0.1, DebounceType.kFalling);

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
    inputs.manipulatorSensorOne = manipulatorOneDetected();
    inputs.manipulatorSensorTwo = manipulatorTwoDetected();
    inputs.funnelSensorOne = funnelOneDetected();
    inputs.funnelSensorTwo = funnelTwoDetected();
    inputs.hasGamePiece = hasGamePiece();
    inputs.gamePieceInFunnel = gamePieceInFunnel();
  }

  private boolean hasGamePiece() {
    return manipulatorOneDetected() && manipulatorTwoDetected();
  }

  private boolean gamePieceInFunnel() {
    return funnelOneDetected() || funnelTwoDetected();
  }

  private boolean manipulatorOneDetected() {
    return m_manipulatorSensorOneDebouncer.calculate(m_manipulatorSensorOne.get());
  }

  private boolean manipulatorTwoDetected() {
    return m_manipulatorSensorTwoDebouncer.calculate(m_manipulatorSensorTwo.get());
  }

  // these photoelectrics give different readings from the manipulator ones
  private boolean funnelOneDetected() {
    return m_funnelSensorOneDebouncer.calculate(!m_funnelSensorOne.get());
  }

  private boolean funnelTwoDetected() {
    return m_funnelSensorTwoDebouncer.calculate(!m_funnelSensorTwo.get());
  }
}
