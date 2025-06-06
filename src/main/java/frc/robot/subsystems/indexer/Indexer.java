package frc.robot.subsystems.indexer;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FullTuningConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.RobotState;
import frc.robot.util.AlertManager;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO m_io;
  public final IndexerInputsAutoLogged m_inputs = new IndexerInputsAutoLogged();

  private Alert m_motorDisconnectedAlert =
      new Alert("Indexer Motor Disconnected", AlertType.kError);

  public static enum IndexerState {
    kIdle,
    kIndexing,
    kCoralEject,
    kFullTuning,
  }

  private SubsystemProfiles<IndexerState> m_profiles;

  public Indexer(IndexerIO io) {
    m_io = io;

    Map<IndexerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IndexerState.kIdle, this::idlePeriodic);
    periodicHash.put(IndexerState.kIndexing, this::indexingPeriodic);
    periodicHash.put(IndexerState.kCoralEject, this::coralEjectPeriodic);
    periodicHash.put(IndexerState.kFullTuning, this::fullTuningPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, IndexerState.kIdle);

    AlertManager.registerAlert(m_motorDisconnectedAlert);
  }

  public void updateState(IndexerState state) {
    if (m_profiles.getCurrentProfile() == IndexerState.kFullTuning) {
      return;
    }

    m_profiles.setCurrentProfile(state);
  }

  public IndexerState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    if (FullTuningConstants.kFullTuningMode) {
      updateState(IndexerState.kFullTuning);
    }

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Indexer", m_inputs);
    Logger.recordOutput("Indexer/State", m_profiles.getCurrentProfile());

    if (Constants.kUseAlerts && !m_inputs.sideMotorIsConnected) {
      m_motorDisconnectedAlert.set(true);
    } else {
      m_motorDisconnectedAlert.set(false);
    }

    Logger.recordOutput("PeriodicTime/Indexer", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void idlePeriodic() {
    m_io.setVoltage(
        IndexerConstants.kIndexerIdleVoltage.get(), IndexerConstants.kIndexerIdleVoltage.get());
  }

  public void indexingPeriodic() {
    if (RobotState.getInstance().getManipulatorAutoReversing()) {
      m_io.setVoltage(
          IndexerConstants.kIndexerEjectVoltage.get(),
          IndexerConstants.kIndexerTopEjectVoltage.get());
    } else {
      m_io.setVoltage(
          IndexerConstants.kIndexerIndexingVoltage.get(),
          IndexerConstants.kIndexerTopIndexingVoltage.get());
    }
  }

  public void coralEjectPeriodic() {
    m_io.setVoltage(
        IndexerConstants.kIndexerEjectVoltage.get(),
        IndexerConstants.kIndexerTopEjectVoltage.get());
  }

  public void fullTuningPeriodic() {
    m_io.setVoltage(
        FullTuningConstants.kIndexerVoltage.get(), FullTuningConstants.kIndexerVoltage.get());
  }
}
