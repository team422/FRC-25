package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private DCMotorSim m_sim;

  private double m_voltage = 0.0;

  public IntakeRollerIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.kRollerSimGearbox,
            IntakeConstants.kRollerSimMOI,
            IntakeConstants.kRollerSimGearing);

    m_sim = new DCMotorSim(plant, IntakeConstants.kRollerSimGearbox);
  }

  @Override
  public void updateInputs(IntakeRollerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.velocityRPS = m_sim.getAngularVelocityRPM() / 60;
    inputs.accelerationRPSSq = Units.radiansToRotations(m_sim.getAngularAccelerationRadPerSecSq());
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;

    // these don't matter in sim
    inputs.statorCurrent = 0.0;
    inputs.temperature = 0.0;
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

  @Override
  public boolean hasGamePiece() {
    // Possibly implement later
    return false;
  }
}
