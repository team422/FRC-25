package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim m_sim;
  private ProfiledPIDController m_controller;
  private ElevatorFeedforward m_feedforward;
  private TrapezoidProfile m_trap;
  private double m_voltage;

  public ElevatorIOSim() {
    Constraints constraints =
        new Constraints(ElevatorConstants.kTopSpeed, ElevatorConstants.kTopAcceleration);

    var plant =
        LinearSystemId.createElevatorSystem(
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kSimMOI,
            ElevatorConstants.kRadius,
            ElevatorConstants.kSimGearing);
    m_sim =
        new ElevatorSim(
            plant,
            ElevatorConstants.kSimGearbox,
            ElevatorConstants.kMinHeight,
            ElevatorConstants.kMaxHeight,
            false,
            ElevatorConstants.kMinHeight);

    m_controller =
        new ProfiledPIDController(
            ElevatorConstants.kP.getAsDouble(),
            ElevatorConstants.kI.getAsDouble(),
            ElevatorConstants.kD.getAsDouble(),
            constraints);
    m_controller.setTolerance(ElevatorConstants.kHeightTolerance);

    m_feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kKS.getAsDouble(),
            ElevatorConstants.kKG.getAsDouble(),
            ElevatorConstants.kKV.getAsDouble(),
            ElevatorConstants.kKA.getAsDouble());
    m_trap = new TrapezoidProfile(constraints);
  }

  @Override
  public void updateInputs(ElevatorInputsAutoLogged inputs) {
    State curr =
        m_trap.calculate(
            .02,
            new State(m_sim.getPositionMeters() / 100, m_sim.getVelocityMetersPerSecond()),
            m_controller.getGoal());
    m_voltage =
        m_controller.calculate(m_sim.getPositionMeters(), curr.position)
            + m_feedforward.calculate(curr.velocity);

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.atSetpoint = m_controller.atGoal();
    inputs.currLocation = m_sim.getPositionMeters();
    inputs.desiredLocation = m_controller.getGoal().position;
    inputs.voltage = 0; // idk what to do here
  }

  @Override
  public void setDesiredHeight(double centimeters) {
    m_controller.setGoal(centimeters);
  }

  @Override
  public void setPIDFF(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    m_controller.setPID(kP, kI, kD);
    m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }
}
