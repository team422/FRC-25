package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;

  public IndexerIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimMOI, IndexerConstants.kSimGearing);

    m_sim = new DCMotorSim(plant, IndexerConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.velocityRPS = m_sim.getAngularVelocityRPM() / 60;
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;

    // these don't matter in sim
    inputs.statorCurrent = 0.0;
    inputs.motorIsConnected = true;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // Not needed for simulation
  }
}
