package frc.robot.subsystems.indexer;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO m_io;
  public final IndexerInputsAutoLogged m_inputs = new IndexerInputsAutoLogged();

  public static enum IndexerState {
    kIdle,
    kIndexing,
  }

  private SubsystemProfiles<IndexerState> m_profiles;

  public Indexer(IndexerIO io) {
    m_io = io;

    Map<IndexerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IndexerState.kIdle, this::idlePeriodic);
    periodicHash.put(IndexerState.kIndexing, this::indexingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, IndexerState.kIdle);
  }

  public void updateState(IndexerState state) {
    m_profiles.setCurrentProfile(state);
  }

  public IndexerState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Indexer", m_inputs);
    Logger.recordOutput("Indexer/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("PeriodicTime/Indexer", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void idlePeriodic() {
    m_io.setVoltage(IndexerConstants.kIndexerIdleVoltage.get());
  }

  public void indexingPeriodic() {
    m_io.setVoltage(IndexerConstants.kIndexerIndexingVoltage.get());
  }
}
