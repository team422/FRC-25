package frc.robot.subsystems.manipulator.wrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ManipulatorConstants;

public class WristIOSim implements WristIO {
  private SingleJointedArmSim m_sim;
  private PIDController m_controller = new PIDController(0, 0, 0);

  public WristIOSim() {
    m_sim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                ManipulatorConstants.kWristSimGearbox,
                ManipulatorConstants.kWristSimMOI,
                ManipulatorConstants.kWristSimGearing),
            ManipulatorConstants.kWristSimGearbox,
            ManipulatorConstants.kWristSimGearing,
            ManipulatorConstants.kWristArmLength,
            ManipulatorConstants.kWristMinAngle.getRadians(),
            ManipulatorConstants.kWristMaxAngle.getRadians(),
            ManipulatorConstants.kSimSimulateGravity,
            ManipulatorConstants.kSimStartingAngle.getRadians());

    m_controller.setTolerance(ManipulatorConstants.kWristTolerance);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    double voltage =
        m_controller.calculate(Rotation2d.fromRadians(m_sim.getAngleRads()).getDegrees());
    m_sim.setInputVoltage(voltage);
    m_sim.update(.02);

    inputs.connected = true;
    inputs.position = Rotation2d.fromRadians(m_sim.getAngleRads()).getDegrees();
    inputs.statorCurrent = m_sim.getCurrentDrawAmps();
    inputs.velocity = m_sim.getVelocityRadPerSec();
    inputs.voltage = voltage;
    inputs.atSetpoint = m_controller.atSetpoint();
    inputs.desired = m_controller.getSetpoint();
  }

  @Override
  public void setAngle(Rotation2d angle) {
    m_controller.setSetpoint(angle.getDegrees());
  }

  @Override
  public void setPIDFF(int slot, double p, double i, double d, double kS) {
    m_controller.setPID(p, i, d);
  }
}
