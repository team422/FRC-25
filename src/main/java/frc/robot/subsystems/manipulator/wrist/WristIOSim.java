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
    var plant =
        LinearSystemId.createSingleJointedArmSystem(
            ManipulatorConstants.kWristSimGearbox,
            ManipulatorConstants.kWristSimMOI,
            ManipulatorConstants.kWristSimGearing);

    m_sim =
        new SingleJointedArmSim(
            plant,
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
    double pidVoltage = m_controller.calculate(getCurrAngle().getDegrees());
    m_sim.setInputVoltage(pidVoltage);
    m_sim.update(0.020);

    inputs.currAngleDeg = getCurrAngle().getDegrees();
    inputs.desiredAngleDeg = m_controller.getSetpoint();
    inputs.atSetpoint = atSetpoint();
    inputs.velocityRPS = m_sim.getVelocityRadPerSec();
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = pidVoltage;

    // these don't matter in sim
    inputs.statorCurrent = 0.0;
    inputs.temperature = 0.0;
    inputs.motorIsConnected = false;
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS) {
    m_controller.setPID(kP, kI, kD);
    // kS is not used in simulation
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    m_controller.setSetpoint(angle.getDegrees());
  }

  private Rotation2d getCurrAngle() {
    return Rotation2d.fromRadians(m_sim.getAngleRads());
  }

  private boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // Not needed for simulation
  }
}
