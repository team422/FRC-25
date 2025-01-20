package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSim implements ClimbIO {

  private SingleJointedArmSim m_sim; // We choose to simulate the climb arm as a single-jointed arm
  private PIDController m_controller = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0, 0);

  private boolean m_positionControl = true;
  private double m_voltage = 0;

  public ClimbIOSim() {
    m_sim =
        new SingleJointedArmSim(
            ClimbConstants.kSimGearbox,
            ClimbConstants.kSimGearing,
            ClimbConstants.kSimMOI,
            ClimbConstants.kSimClimbArmLengthMeters,
            ClimbConstants.kSimMinAngleRad,
            ClimbConstants.kSimMaxAngleRad,
            ClimbConstants.kSimGravity,
            ClimbConstants.kSimStartingAngleRad); // init angle 90 deg
    m_controller.setTolerance(ClimbConstants.kClimbTolerance.getRadians()); // radians
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    //use voltage calc'ed from controller, else use set voltage
    if (m_positionControl) {
      double pidVoltage = m_controller.calculate(m_sim.getAngleRads());
      double feedforwardVoltage = m_feedforward.calculate(m_controller.getSetpoint());
      m_voltage = pidVoltage + feedforwardVoltage;
    }

    //run sim for a dt
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    //update inputs
    inputs.atSetpoint = m_controller.atSetpoint();
    inputs.currPositionRad = m_sim.getAngleRads();
    inputs.desiredPositionRad = m_controller.getSetpoint();
    inputs.positionControl = m_positionControl;
    inputs.voltage = m_voltage;
    inputs.current = m_feedforward.calculate(m_sim.getVelocityRadPerSec());
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kV) {
    m_controller.setPID(kP, kI, kD);
    m_feedforward = new SimpleMotorFeedforward(kS, kV);
  }

  @Override
  public void setVoltage(double voltage) {
    m_positionControl = false;
    m_voltage = voltage;
    m_sim.setInputVoltage(voltage);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    m_positionControl = true;
    m_controller.setSetpoint(angle.getRadians());
  }
}
