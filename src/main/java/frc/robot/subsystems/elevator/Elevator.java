package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorInputsAutoLogged m_inputs;
  private ElevatorIO m_io;
  private int m_currPort;
  private double m_desiredHeight;

  private static enum ElevatorState {
    kStow,
    kScoring,
    kIntaking,
    kKnocking,
  }

  private SubsystemProfiles<ElevatorState> m_states;

  public Elevator(ElevatorIO io) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();
    m_io.setPIDFF(
        0,
        ElevatorConstants.kP.getAsDouble(),
        ElevatorConstants.kI.getAsDouble(),
        ElevatorConstants.kD.getAsDouble(),
        ElevatorConstants.kKS.getAsDouble(),
        ElevatorConstants.kKV.getAsDouble(),
        ElevatorConstants.kKA.getAsDouble(),
        ElevatorConstants.kKG.getAsDouble());

    Map<ElevatorState, Runnable> periodicPicker = new HashMap<>();
    periodicPicker.put(ElevatorState.kStow, () -> stowPeriodic());
    periodicPicker.put(ElevatorState.kScoring, () -> scoringPeriodic());
    periodicPicker.put(ElevatorState.kIntaking, () -> intakingPeriodic());
    periodicPicker.put(ElevatorState.kKnocking, () -> knockingPeriodic());

    m_states = new SubsystemProfiles<ElevatorState>(periodicPicker, ElevatorState.kStow);
  }

  @Override
  public void periodic() {
    double startingTime = Timer.getFPGATimestamp();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            m_io.setPIDFF(
                m_currPort,
                ElevatorConstants.kP.getAsDouble(),
                ElevatorConstants.kI.getAsDouble(),
                ElevatorConstants.kD.getAsDouble(),
                ElevatorConstants.kKS.getAsDouble(),
                ElevatorConstants.kKV.getAsDouble(),
                ElevatorConstants.kKA.getAsDouble(),
                ElevatorConstants.kKG.getAsDouble()),
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.kKS,
        ElevatorConstants.kKV,
        ElevatorConstants.kKA,
        ElevatorConstants.kKG);
    m_io.updateInputs(m_inputs);
    m_states.getPeriodicFunction().run();

    Logger.recordOutput("ElevatorState", m_states.getCurrentProfile());
    Logger.recordOutput("Timing/elevatorPeriodic", Timer.getFPGATimestamp() - startingTime);
  }

  public void changeState(ElevatorState state) {
    switch (state) {
      case kStow:
        m_states.setCurrentProfile(ElevatorState.kStow);
      case kScoring:
        m_states.setCurrentProfile(ElevatorState.kScoring);
      case kIntaking:
        m_states.setCurrentProfile(ElevatorState.kIntaking);
      case kKnocking:
        m_states.setCurrentProfile(ElevatorState.kKnocking);
    }
  }

  public ElevatorState getState() {
    return m_states.getCurrentProfile();
  }

  public void setDesiredHeight(double desired) {
    m_desiredHeight = desired;
  }

  public void stowPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kStowHeight);
  }

  public void scoringPeriodic() {
    m_io.setDesiredHeight(m_desiredHeight);
  }

  public void intakingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kIntakingHeight);
  }

  public void knockingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kKnockingHeight);
  }
}
