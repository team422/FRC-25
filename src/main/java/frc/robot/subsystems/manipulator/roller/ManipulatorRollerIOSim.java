package frc.robot.subsystems.manipulator.roller;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorRollerIOSim implements ManipulatorRollerIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;

  public ManipulatorRollerIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            ManipulatorConstants.kRollerSimGearbox,
            ManipulatorConstants.kRollerSimMOI,
            ManipulatorConstants.kRollerSimGearing);

    m_sim = new DCMotorSim(plant, ManipulatorConstants.kRollerSimGearbox);
  }

  @Override
  public void updateInputs(ManipulatorRollerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;
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
