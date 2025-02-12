package frc.robot.subsystems.elevator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FullTuningConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public final ElevatorInputsAutoLogged m_inputs;
  private ElevatorIO m_io;
  private double m_desiredHeight;

  public static enum ElevatorState {
    kStow,
    kScoring,
    kIntaking,
    kKnocking,
    kFullTuning,
  }

  private SubsystemProfiles<ElevatorState> m_profiles;

  public Elevator(ElevatorIO io) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();
    m_io.setPIDFF(
        0,
        ElevatorConstants.kP0.getAsDouble(),
        ElevatorConstants.kI.getAsDouble(),
        ElevatorConstants.kD.getAsDouble(),
        ElevatorConstants.kKS.getAsDouble(),
        ElevatorConstants.kKV0.getAsDouble(),
        ElevatorConstants.kKA.getAsDouble(),
        ElevatorConstants.kKG0.getAsDouble());

    Map<ElevatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ElevatorState.kStow, this::stowPeriodic);
    periodicHash.put(ElevatorState.kScoring, this::scoringPeriodic);
    periodicHash.put(ElevatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ElevatorState.kKnocking, this::knockingPeriodic);
    periodicHash.put(ElevatorState.kFullTuning, this::fullTuningPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, ElevatorState.kStow);

    // to set the setpoint
    updateState(ElevatorState.kStow);

    m_io.setMagic(
        ElevatorConstants.kMagicMotionCruiseVelocity.get(),
        ElevatorConstants.kMagicMotionAcceleration.get(),
        ElevatorConstants.kMagicMotionJerk.get());
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    if (FullTuningConstants.kFullTuningMode) {
      updateState(ElevatorState.kFullTuning);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_io.setPIDFF(
              0,
              ElevatorConstants.kP0.getAsDouble(),
              ElevatorConstants.kI.getAsDouble(),
              ElevatorConstants.kD.getAsDouble(),
              ElevatorConstants.kKS.getAsDouble(),
              ElevatorConstants.kKV0.getAsDouble(),
              ElevatorConstants.kKA.getAsDouble(),
              ElevatorConstants.kKG0.getAsDouble());

          m_io.setPIDFF(
              1,
              ElevatorConstants.kP1.getAsDouble(),
              ElevatorConstants.kI.getAsDouble(),
              ElevatorConstants.kD.getAsDouble(),
              ElevatorConstants.kKS.getAsDouble(),
              ElevatorConstants.kKV1.getAsDouble(),
              ElevatorConstants.kKA.getAsDouble(),
              ElevatorConstants.kKG1.getAsDouble());

          m_io.setPIDFF(
              2,
              ElevatorConstants.kP2.getAsDouble(),
              ElevatorConstants.kI.getAsDouble(),
              ElevatorConstants.kD.getAsDouble(),
              ElevatorConstants.kKS.getAsDouble(),
              ElevatorConstants.kKV2.getAsDouble(),
              ElevatorConstants.kKA.getAsDouble(),
              ElevatorConstants.kKG2.getAsDouble());
        },
        ElevatorConstants.kP0,
        ElevatorConstants.kP1,
        ElevatorConstants.kP2,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.kKS,
        ElevatorConstants.kKV0,
        ElevatorConstants.kKV1,
        ElevatorConstants.kKV2,
        ElevatorConstants.kKA,
        ElevatorConstants.kKG0,
        ElevatorConstants.kKG1,
        ElevatorConstants.kKG2);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_io.setMagic(
              Units.inchesToMeters(ElevatorConstants.kMagicMotionCruiseVelocity.get()),
              Units.inchesToMeters(ElevatorConstants.kMagicMotionAcceleration.get()),
              Units.inchesToMeters(ElevatorConstants.kMagicMotionJerk.get()));
        },
        ElevatorConstants.kMagicMotionCruiseVelocity,
        ElevatorConstants.kMagicMotionAcceleration,
        ElevatorConstants.kMagicMotionJerk);

    m_io.updateInputs(m_inputs);

    double currHeight = m_io.getCurrHeight();
    if (currHeight < Units.inchesToMeters(18)) {
      m_io.setSlot(0);
    } else if (currHeight < Units.inchesToMeters(45)) {
      m_io.setSlot(1);
    } else {
      m_io.setSlot(2);
    }

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Elevator", m_inputs);
    Logger.recordOutput("Elevator/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("PeriodicTime/Elevator", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(ElevatorState state) {
    if (m_profiles.getCurrentProfile() == ElevatorState.kFullTuning) {
      // if we are in full tuning mode we don't want to change the state
      return;
    }

    m_profiles.setCurrentProfile(state);
    switch (state) {
      case kIntaking:
        m_io.setDesiredHeight(ElevatorConstants.kIntakingHeight);
        break;
      case kKnocking:
        m_io.setDesiredHeight(ElevatorConstants.kKnockingHeight);

        break;
      case kScoring:
        m_io.setDesiredHeight(m_desiredHeight);

        break;
      case kStow:
        m_io.setDesiredHeight(ElevatorConstants.kStowHeight);

        break;
      case kFullTuning:
        m_io.setDesiredHeight(Units.inchesToMeters(FullTuningConstants.kElevatorSetpoint.get()));

        break;
    }
  }

  public ElevatorState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredHeight(double desired) {
    m_desiredHeight = desired;
    if (m_profiles.getCurrentProfile() == ElevatorState.kScoring) {
      m_io.setDesiredHeight(m_desiredHeight);
    }
  }

  public double getDesiredHeight() {
    return m_desiredHeight;
  }

  public double getCurrHeight() {
    return m_io.getCurrHeight();
  }

  public void stowPeriodic() {}

  public void scoringPeriodic() {}

  public void intakingPeriodic() {}

  public void knockingPeriodic() {}

  public void fullTuningPeriodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_io.setDesiredHeight(Units.inchesToMeters(FullTuningConstants.kElevatorSetpoint.get()));
        },
        FullTuningConstants.kElevatorSetpoint);
  }

  public boolean atSetpoint() {
    return m_io.atSetpoint();
  }
}
