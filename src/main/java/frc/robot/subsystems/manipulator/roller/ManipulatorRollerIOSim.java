package frc.robot.subsystems.manipulator.roller;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorRollerIOSim implements ManipulatorRollerIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;

  private PIDController m_positionController = new PIDController(0.0, 0.0, 0.0);
  private boolean m_positionControl = false;

  public ManipulatorRollerIOSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            ManipulatorConstants.kRollerSimGearbox,
            ManipulatorConstants.kRollerSimMOI,
            ManipulatorConstants.kRollerSimGearing);

    m_sim = new DCMotorSim(plant, ManipulatorConstants.kRollerSimGearbox);

    m_positionController.setTolerance(ManipulatorConstants.kRollerPositionTolerance);
  }

  @Override
  public void updateInputs(ManipulatorRollerInputs inputs) {
    if (m_positionControl) {
      m_voltage = m_positionController.calculate(getPosition().in(Degrees));
    }

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.positionDegrees = getPosition().in(Degrees);
    inputs.positionControl = m_positionControl;
    inputs.desiredPositionDegrees = m_positionController.getSetpoint();
    inputs.velocityRPS = m_sim.getAngularVelocityRPM() / 60.0;
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;

    // these don't matter in sim
    inputs.statorCurrent = 0.0;
    inputs.temperature = 0.0;
    inputs.motorIsConnected = false;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_positionControl = false;
  }

  @Override
  public void setDesiredPosition(Angle position) {
    m_positionControl = true;
    m_positionController.setSetpoint(position.in(Degrees));
  }

  @Override
  public Angle getPosition() {
    return Radians.of(m_sim.getAngularPositionRad());
  }

  @Override
  public void setPositionPID(double kP, double kI, double kD, double kS) {
    m_positionController.setPID(kP, kI, kD);
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // Not needed for simulation
  }
}
