package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOSim implements ClimbIO {

  private SingleJointedArmSim m_sim; // We choose to simulate the climb arm as a single-jointed arm
  private PIDController m_controller = new PIDController(0, 0, 0);

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
    m_controller.setTolerance(ClimbConstants.kClimbTolerance);
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    double m_voltage = m_controller.calculate(Units.radiansToDegrees(m_sim.getAngleRads()));

    // run sim for a dt
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    // update inputs
    inputs.atSetpoint = m_controller.atSetpoint();
    inputs.currPositionDegrees = getCurrPosition().getDegrees();
    inputs.desiredPositionDegrees = m_controller.getSetpoint();
    inputs.voltage = m_voltage;
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.motorIsConnected = true;
  }

  @Override
  public void setPID(int slot, double kP, double kI, double kD) {
    m_controller.setPID(kP, kI, kD);
  }

  @Override
  public void setSlot(int slot) {
    // Not used in simulation
  }

  @Override
  public void setDesiredAngle(Rotation2d angle, double feedforward) {
    // feedforward is not used in simulation
    m_controller.setSetpoint(angle.getDegrees());
  }

  @Override
  public void zeroEncoder() {
    m_sim.setState(0, 0);
  }

  public Rotation2d getCurrPosition() {
    return Rotation2d.fromRadians(m_sim.getAngleRads());
  }
}
