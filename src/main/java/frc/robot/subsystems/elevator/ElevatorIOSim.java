package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim m_sim;
  private PIDController m_controller = new PIDController(0, 0, 0);
  private double m_desiredHeight;
  private double m_kG;

  public ElevatorIOSim() {
    m_sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                ElevatorConstants.kSimGearbox,
                ElevatorConstants.kSimMOI,
                ElevatorConstants.kDiameter / 2,
                ElevatorConstants.kSimGearing),
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kMinHeight,
            ElevatorConstants.kMaxHeight,
            true,
            ElevatorConstants.kMinHeight);

    m_controller.setTolerance(ElevatorConstants.kHeightTolerance);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    m_sim.update(.02);
    double voltage = m_controller.calculate(m_sim.getPositionMeters(), m_desiredHeight) + m_kG;
    m_sim.setInputVoltage(voltage);
    inputs.desiredHeight = m_desiredHeight;
    inputs.leadingHeight = m_sim.getPositionMeters();
    inputs.followingHeight = m_sim.getPositionMeters();
    inputs.atSetpoint = m_controller.atSetpoint();
    inputs.leadingMotorConnected = true;
    inputs.followingMotorConnected = true;
    inputs.leadingVoltage = voltage;
  }

  @Override
  public void setHeight(double height) {
    m_desiredHeight = height;
  }

  @Override
  public void setPIDFF(int slot, double p, double i, double d, double kG) {
    m_controller.setPID(p, i, d);
    m_kG = kG;
  }
}
