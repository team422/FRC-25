package frc.robot.subsystems.manipulator.detector;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralDetectorIOReal implements CoralDectectorIO {
  private DigitalInput m_manipOne;
  private DigitalInput m_manipTwo;
  private DigitalInput m_funnelOne;
  private DigitalInput m_funnelTwo;

  public CoralDetectorIOReal(int manipOne, int manipTwo, int funnelOne, int funnelTwo) {
    m_manipOne = new DigitalInput(manipOne);
    m_manipTwo = new DigitalInput(manipTwo);
    m_funnelOne = new DigitalInput(funnelOne);
    m_funnelTwo = new DigitalInput(funnelTwo);
  }

  @Override
  public void updateInputs(DetectorInputs inputs) {
    inputs.funnelSensorOne = m_funnelOne.get();
    inputs.funnelSensorTwo = m_funnelTwo.get();
    inputs.manipSensorOne = m_manipOne.get();
    inputs.manipSensorTwo = m_manipTwo.get();
    inputs.hasGamePiece = hasGamePiece();
    inputs.gamePieceInFunnel = pieceInFunnel();
  }

  public boolean hasGamePiece() {
    return m_manipOne.get() || m_manipTwo.get();
  }

  public boolean pieceInFunnel() {
    return m_funnelOne.get() || m_funnelTwo.get();
  }
}
