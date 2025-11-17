package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO m_io;
  private IntakeInputsAutoLogged m_inputs;
  private SubsystemProfiles<IntakeState> m_profiles;

  public enum IntakeState {
    kIdle,
    kIntaking
  }

  public Intake(IntakeIO io) {
    m_io = io;
    m_inputs = new IntakeInputsAutoLogged();

    HashMap<IntakeState, Runnable> hash = new HashMap<>();
    hash.put(IntakeState.kIdle, this::idlePeriodic);
    hash.put(IntakeState.kIntaking, this::intakingPeriodic);
    m_profiles = new SubsystemProfiles<>(hash, IntakeState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.processInputs("Intake", m_inputs);
    Logger.recordOutput("Intake/IntakeState", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(
        IndexerConstants.kIndexerIdleVoltage.get(), IndexerConstants.kIndexerIdleVoltage.get());
  }

  public void intakingPeriodic() {
    m_io.setVoltage(
        IndexerConstants.kIndexerTopIndexingVoltage.get(),
        IndexerConstants.kIndexerIndexingVoltage.get());
  }

  public IntakeState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void updateState(IntakeState state) {
    m_profiles.setCurrentProfile(state);
  }
}
