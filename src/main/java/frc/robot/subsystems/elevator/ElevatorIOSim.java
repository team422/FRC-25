package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  // HELLO
  // every time it says meters, it means inches
  // all the methods are called meters for the sim but the radius is in inches

  private ElevatorSim m_sim;
  private PIDController m_controller;
  private double m_voltage;
  private double m_desiredHeight;
  private Timer m_timer = new Timer();
  private boolean m_positionControl = true;
  private double m_kG;

  public ElevatorIOSim() {
    var plant =
        LinearSystemId.createElevatorSystem(
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kSimMOI,
            ElevatorConstants.kDiameter / 2,
            ElevatorConstants.kSimGearing);
    m_sim =
        new ElevatorSim(
            plant,
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kMinHeight,
            ElevatorConstants.kMaxHeight,
            true,
            ElevatorConstants.kMinHeight);

    m_controller = new PIDController(0, 0, 0);
    m_controller.setTolerance(ElevatorConstants.kHeightTolerance);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    if (m_positionControl) {
      m_voltage = m_controller.calculate(m_sim.getPositionMeters(), m_desiredHeight) + m_kG;
      Logger.recordOutput("Elevator/DesiredPosition", m_desiredHeight);
    }
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.leadingVelocity = m_sim.getVelocityMetersPerSecond();
    inputs.atSetpoint = atSetpoint();
    inputs.leadingPosition = m_sim.getPositionMeters();
    inputs.desiredLocation = m_desiredHeight;
    inputs.leadingVoltage = m_voltage;
    inputs.positionControl = m_positionControl;
  }

  @Override
  public void setDesiredHeight(double inches) {
    m_desiredHeight = inches;
    m_timer.restart();
    m_positionControl = true;
  }

  @Override
  public void setPIDFF(int slot, double kP, double kI, double kD, double kG) {
    m_controller.setPID(kP, kI, kD);
    m_kG = kG;
  }

  @Override
  public void setSlot(int slot) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  private boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public void zeroElevator() {
    // not needed for sim
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
    m_positionControl = false;
  }
}
