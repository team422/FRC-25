package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

public class PivotIOSim implements PivotIO {
  private SingleJointedArmSim m_sim;
  private PIDController m_controller =
      new PIDController(
          IntakeConstants.kPivotSimP, IntakeConstants.kPivotSimI, IntakeConstants.kPivotSimD);
  private double m_kG = 0.0;

  public PivotIOSim() {
    var plant =
        LinearSystemId.createSingleJointedArmSystem(
            IntakeConstants.kPivotSimGearbox,
            IntakeConstants.kPivotSimMOI,
            IntakeConstants.kPivotSimGearing);

    m_sim =
        new SingleJointedArmSim(
            plant,
            IntakeConstants.kPivotSimGearbox,
            IntakeConstants.kPivotSimGearing,
            IntakeConstants.kPivotArmLength,
            IntakeConstants.kPivotMinAngle.getRadians(),
            IntakeConstants.kPivotMaxAngle.getRadians(),
            IntakeConstants.kSimSimulateGravity,
            IntakeConstants.kSimStartingAngle.getRadians());

    m_controller.setTolerance(IntakeConstants.kPivotTolerance);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    double pidVoltage = m_controller.calculate(getCurrAngle().getDegrees());
    double feedforwardVoltage = m_kG * Math.cos(getCurrAngle().getRadians());

    m_sim.setInputVoltage(pidVoltage + feedforwardVoltage);
    m_sim.update(0.020);

    inputs.currAngleDeg = getCurrAngle().getDegrees();
    inputs.desiredAngleDeg = m_controller.getSetpoint();
    inputs.atSetpoint = atSetpoint();
    inputs.velocityRPS = Units.radiansToRotations(m_sim.getVelocityRadPerSec());
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = pidVoltage + feedforwardVoltage;

    // these don't matter in sim
    inputs.statorCurrent = 0.0;
    inputs.temperature = 0.0;
    inputs.motorIsConnected = false;
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kG) {
    m_controller.setPID(kP, kI, kD);
    // kS is not used in simulation
    m_kG = kG;
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    m_controller.setSetpoint(angle.getDegrees());
  }

  @Override
  public Rotation2d getCurrAngle() {
    return Rotation2d.fromRadians(m_sim.getAngleRads());
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    // Not needed for simulation
  }
}
