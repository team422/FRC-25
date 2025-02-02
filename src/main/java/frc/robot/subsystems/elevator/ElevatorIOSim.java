package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim m_sim;
  private PIDController m_controller;
  private ElevatorFeedforward m_feedforward;
  private TrapezoidProfile m_trap;
  private double m_voltage;
  private double m_desiredHeight;
  private State m_startingState = new State();
  private Timer m_timer = new Timer();

  public ElevatorIOSim() {
    Constraints constraints =
        new Constraints(
            ElevatorConstants.kMagicMotionCruiseVelocity.get(),
            ElevatorConstants.kMagicMotionAcceleration.get());

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

    // ask kent if he wants normal or profiled
    m_controller =
        new PIDController(
            ElevatorConstants.kP.getAsDouble(),
            ElevatorConstants.kI.getAsDouble(),
            ElevatorConstants.kD.getAsDouble());
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
  public void updateInputs(ElevatorInputs inputs) {
    State desired = m_trap.calculate(m_timer.get(), m_startingState, new State(m_desiredHeight, 0));
    m_voltage =
        m_controller.calculate(m_sim.getPositionMeters(), desired.position)
            + m_feedforward.calculate(desired.velocity, 0); // could be incorrect

    Logger.recordOutput("Elevator/DesiredPosition", desired.position);
    Logger.recordOutput("Elevator/DesiredVelocity", desired.velocity);
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(.02);

    inputs.leadingVelocity = m_sim.getVelocityMetersPerSecond();
    inputs.atSetpoint = atSetpoint();
    inputs.leadingPosition = m_sim.getPositionMeters();
    inputs.desiredLocation = m_desiredHeight;
    inputs.leadingVoltage = m_voltage;
  }

  @Override
  public void setDesiredHeight(double meters) {
    m_desiredHeight = meters;
    m_startingState = new State(m_sim.getPositionMeters(), m_sim.getVelocityMetersPerSecond());
    m_timer.restart();
  }

  @Override
  public void setPIDFF(
      int slot, double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    m_controller.setPID(kP, kI, kD);
    m_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void setSlot(int slot) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public void setMagic(double velocity, double acceleration, double jerk) {
    m_trap = new TrapezoidProfile(new Constraints(velocity, acceleration));
  }

  @Override
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  @Override
  public double getCurrHeight() {
    return m_sim.getPositionMeters();
  }
}
