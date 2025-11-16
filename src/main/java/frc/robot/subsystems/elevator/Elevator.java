package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_io;
  private double m_desiredHeight;
  private ElevatorInputsAutoLogged m_inputs;
  private SubsystemProfiles<ElevatorState> m_profiles;

  public enum ElevatorState {
    kScoring,
    kStow
  }

  public Elevator(ElevatorIO io) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();
    m_desiredHeight = ElevatorConstants.kL1.get();

    HashMap<ElevatorState, Runnable> m_hash = new HashMap<>();
    m_hash.put(ElevatorState.kStow, this::stowPeriodic);
    m_hash.put(ElevatorState.kScoring, this::scoringPeriodic);

    m_profiles = new SubsystemProfiles<ElevatorState>(m_hash, ElevatorState.kStow);

    if (RobotBase.isReal()) {
      m_io.setPIDFF(
          0,
          ElevatorConstants.kP0.getAsDouble(),
          ElevatorConstants.kI.getAsDouble(),
          ElevatorConstants.kD.getAsDouble(),
          ElevatorConstants.kKG0.getAsDouble());
    } else {
      m_io.setPIDFF(
          0,
          ElevatorConstants.kSimElevatorP,
          ElevatorConstants.kSimElevatorI,
          ElevatorConstants.kSimElevatorD,
          ElevatorConstants.kSimElevatorkG);
    }

    updateState(ElevatorState.kStow);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_io.setPIDFF(
                0,
                ElevatorConstants.kP0.getAsDouble(),
                ElevatorConstants.kI.getAsDouble(),
                ElevatorConstants.kD.getAsDouble(),
                ElevatorConstants.kKG0.getAsDouble());
          }
        },
        ElevatorConstants.kP0,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.kKG0);

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Elevator", m_inputs);
    Logger.recordOutput("Elevator/state", m_profiles.getCurrentProfile());
  }

  public void stowPeriodic() {
    m_io.setHeight(ElevatorConstants.kStowHeight.getAsDouble());
  }

  public void scoringPeriodic() {
    m_io.setHeight(m_desiredHeight);
  }

  public void updateState(ElevatorState state) {
    m_profiles.setCurrentProfile(state);
  }

  public ElevatorState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void setHeight(double height) {
    m_desiredHeight = height;
  }

  public boolean atSetpoint(double height) {
    return Math.abs(m_inputs.leadingHeight - height) < ElevatorConstants.kMaxSkip
        && Math.abs(m_desiredHeight - height) < ElevatorConstants.kMaxSkip;
  }

  public double getDesiredHeight() {
    return m_desiredHeight;
  }

  public double getVelocity() {
    return m_inputs.leadingVelocity;
  }

  public void zeroElevator() {
    m_io.zero();
  }
}
