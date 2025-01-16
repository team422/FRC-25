package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim m_sim;
  private PIDController m_controller = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);

  private double m_voltage = 0;
  private boolean m_velocityControl = false;

  public FlywheelIOSim() {
    var plant =
        LinearSystemId.createFlywheelSystem(
            FlywheelConstants.kSimGearbox,
            FlywheelConstants.kSimMOI,
            FlywheelConstants.kSimGearing);
    m_sim = new FlywheelSim(plant, FlywheelConstants.kSimGearbox);

    m_controller.setTolerance(FlywheelConstants.kVelocityTolerance);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    if (m_velocityControl) {
      double pidVoltage = m_controller.calculate(m_sim.getAngularVelocityRPM() / 60);
      double ffVoltage = m_feedforward.calculate(m_controller.getSetpoint());
      m_voltage = pidVoltage + ffVoltage;
    }

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.currVelocityRPS = m_sim.getAngularVelocityRPM() / 60;
    inputs.desiredVelocityRPS = m_controller.getSetpoint();
    inputs.atSetpoint = m_controller.atSetpoint();
    inputs.velocityControl = m_velocityControl;
    inputs.voltage = m_voltage;
    inputs.current = m_sim.getCurrentDrawAmps();
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kV) {
    m_controller.setPID(kP, kI, kD);
    m_feedforward = new SimpleMotorFeedforward(kS, kV);
  }

  @Override
  public void setDesiredVelocity(double velocityRPS) {
    m_velocityControl = true;
    m_controller.setSetpoint(velocityRPS);
  }

  @Override
  public void setVoltage(double voltage) {
    m_velocityControl = false;
    m_voltage = voltage;
    m_controller.setSetpoint(0);
  }
}
