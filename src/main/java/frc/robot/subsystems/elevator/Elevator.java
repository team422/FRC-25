package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorInputsAutoLogged m_inputs;
  private ElevatorIO m_io;
  private int m_currSlot;
  private FieldConstants.ReefHeight m_desiredLocation;

  private static enum ElevatorState {
    kStow,
    kScoring,
    kIntaking,
    kKnocking,
  }

  private SubsystemProfiles<ElevatorState> m_profiles;

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

    Map<ElevatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ElevatorState.kStow, () -> stowPeriodic());
    periodicHash.put(ElevatorState.kScoring, () -> scoringPeriodic());
    periodicHash.put(ElevatorState.kIntaking, () -> intakingPeriodic());
    periodicHash.put(ElevatorState.kKnocking, () -> knockingPeriodic());

    m_profiles = new SubsystemProfiles<>(periodicHash, ElevatorState.kStow);
  }

  @Override
  public void periodic() {
    double startingTime = Timer.getFPGATimestamp();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            m_io.setPIDFF(
                m_currSlot,
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
    m_profiles.getPeriodicFunction().run();

    Logger.recordOutput("ElevatorState", m_profiles.getCurrentProfile());
    Logger.recordOutput("Timing/elevatorPeriodic", Timer.getFPGATimestamp() - startingTime);
  }

  public void updateState(ElevatorState state) {
    m_profiles.setCurrentProfile(state);
  }

  public ElevatorState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredScoringLocation(FieldConstants.ReefHeight desired) {
    m_desiredLocation = desired;
  }

  public void stowPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kStowHeight);
  }

  public void scoringPeriodic() {
    double desiredHeight = m_desiredLocation.height + ElevatorConstants.kElevatorOffset.get();
    m_io.setDesiredHeight(desiredHeight);
  }

  public void intakingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kIntakingHeight);
  }

  public void knockingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kKnockingHeight);
  }
}
