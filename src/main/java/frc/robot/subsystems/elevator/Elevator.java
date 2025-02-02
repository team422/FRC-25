package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public final ElevatorInputsAutoLogged m_inputs;
  private ElevatorIO m_io;
  private int m_currSlot;
  private FieldConstants.ReefHeight m_desiredLocation = FieldConstants.ReefHeight.L1;

  public static enum ElevatorState {
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
    periodicHash.put(ElevatorState.kStow, this::stowPeriodic);
    periodicHash.put(ElevatorState.kScoring, this::scoringPeriodic);
    periodicHash.put(ElevatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ElevatorState.kKnocking, this::knockingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, ElevatorState.kStow);

    updateState(ElevatorState.kStow);

    m_io.setMagic(20, 2, 1);
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

    Logger.processInputs("Elevator", m_inputs);
    Logger.recordOutput("Elevator/DesiredLocation", m_desiredLocation);
    Logger.recordOutput("Elevator/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("PeriodicTime/Elevator", Timer.getFPGATimestamp() - startingTime);
  }

  public void updateState(ElevatorState state) {
    m_profiles.setCurrentProfile(state);
    switch (state) {
      case kIntaking:
        m_io.setDesiredHeight(ElevatorConstants.kIntakingHeight);
        break;
      case kKnocking:
        m_io.setDesiredHeight(ElevatorConstants.kKnockingHeight);

        break;
      case kScoring:
        double desiredHeight = m_desiredLocation.height + ElevatorConstants.kElevatorOffset.get();
        m_io.setDesiredHeight(desiredHeight);

        break;
      case kStow:
        m_io.setDesiredHeight(ElevatorConstants.kStowHeight);

        break;
    }
  }

  public ElevatorState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredScoringLocation(FieldConstants.ReefHeight desired) {
    m_desiredLocation = desired;
    if (m_profiles.getCurrentProfile() == ElevatorState.kScoring) {
      double desiredHeight = m_desiredLocation.height + ElevatorConstants.kElevatorOffset.get();
      m_io.setDesiredHeight(desiredHeight);
    }
  }

  public FieldConstants.ReefHeight getDesiredScoringLocation() {
    return m_desiredLocation;
  }

  public void stowPeriodic() {}

  public void scoringPeriodic() {}

  public void intakingPeriodic() {}

  public void knockingPeriodic() {}

  public boolean atSetpoint() {
    return m_io.atSetpoint();
  }
}
