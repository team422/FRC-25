package frc.robot.subsystems.manipulator.rollers;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorConstants;

public class RollerIOSim implements RollerIO {
  private DCMotorSim m_sim;
  private double m_voltage;

  public RollerIOSim() {
    m_sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ManipulatorConstants.kRollerSimGearbox,
                ManipulatorConstants.kRollerSimMOI,
                ManipulatorConstants.kRollerSimGearing),
            ManipulatorConstants.kRollerSimGearbox);
  }

  @Override
  public void setVoltage(double volts) {
    m_voltage = volts;
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    m_sim.update(.02);
    m_sim.setInputVoltage(m_voltage);

    inputs.velocity = m_sim.getAngularVelocityRPM() / 60;
    inputs.voltage = m_voltage;
    inputs.connected = true;
  }
}
