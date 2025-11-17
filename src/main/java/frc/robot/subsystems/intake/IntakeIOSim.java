package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_topSim;
  private DCMotorSim m_sideSim;
  private double m_topVoltage;
  private double m_sideVoltage;

  public IntakeIOSim() {
    m_topSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IndexerConstants.kSimGearbox,
                IndexerConstants.kSimMOI,
                IndexerConstants.kSimGearing),
            IndexerConstants.kSimGearbox);
    m_sideSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                IndexerConstants.kSimGearbox,
                IndexerConstants.kSimMOI,
                IndexerConstants.kSimGearing),
            IndexerConstants.kSimGearbox);
    m_topVoltage = 0;
    m_sideVoltage = 0;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    m_topSim.setInputVoltage(m_topVoltage);
    m_topSim.update(.02);
    m_sideSim.setInputVoltage(m_sideVoltage);
    m_sideSim.update(.02);

    inputs.topConnected = true;
    inputs.topStatorCurrent = m_topSim.getCurrentDrawAmps();
    inputs.topSupplyCurrent = m_sideSim.getCurrentDrawAmps();
    inputs.topVelocity = m_topSim.getAngularVelocityRPM() / 60;
    inputs.topVoltage = m_topVoltage;

    inputs.sideConnected = true;
    inputs.sideStatorCurrent = m_sideSim.getCurrentDrawAmps();
    inputs.sideSupplyCurrent = m_sideSim.getCurrentDrawAmps();
    inputs.sideVelocity = m_sideSim.getAngularVelocityRPM() / 60;
    inputs.sideVoltage = m_sideVoltage;
  }

  @Override
  public void setVoltage(double top, double side) {
    m_topVoltage = top;
    m_sideVoltage = side;
  }
}
