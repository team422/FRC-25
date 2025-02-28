package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.Constants.FullTuningConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIO;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorInputsAutoLogged;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIO;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.wrist.WristIO;
import frc.robot.subsystems.manipulator.wrist.WristInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private ManipulatorRollerIO m_rollerIO;
  private WristIO m_wristIO;
  private CoralDetectorIO m_coralDetectorIO;

  private Alert m_wristMotorDisconnectedAlert =
      new Alert("Manipulator Wrist Motor Disconnected", AlertType.kError);
  private Alert m_rollerMotorDisconnectedAlert =
      new Alert("Manipulator Roller Motor Disconnected", AlertType.kError);

  private Rotation2d m_desiredWristAngle = new Rotation2d();
  private boolean m_runRollerScoring = false;
  private boolean m_runRollerAlgaeDescoring = false;

  public final ManipulatorRollerInputsAutoLogged m_rollerInputs =
      new ManipulatorRollerInputsAutoLogged();
  public final WristInputsAutoLogged m_wristInputs = new WristInputsAutoLogged();
  public final CoralDetectorInputsAutoLogged m_coralDetectorInputs =
      new CoralDetectorInputsAutoLogged();

  public static enum ManipulatorState {
    kStow,
    kIntaking,
    kIndexing,
    kScoring,
    kAlgaeDescoring,
    kAlgaeHold,
    kFullTuning,
  }

  private SubsystemProfiles<ManipulatorState> m_profiles;

  public Manipulator(
      ManipulatorRollerIO rollerIO, WristIO wristIO, CoralDetectorIO coralDetectorIO) {
    m_rollerIO = rollerIO;
    m_wristIO = wristIO;
    m_coralDetectorIO = coralDetectorIO;

    Map<ManipulatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ManipulatorState.kStow, this::stowPeriodic);
    periodicHash.put(ManipulatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ManipulatorState.kIndexing, this::indexingPeriodic);
    periodicHash.put(ManipulatorState.kScoring, this::scoringPeriodic);
    periodicHash.put(ManipulatorState.kAlgaeDescoring, this::algaeDescoringPeriodic);
    periodicHash.put(ManipulatorState.kAlgaeHold, this::algaeHoldPeriodic);
    periodicHash.put(ManipulatorState.kFullTuning, this::fullTuningPeriodic);
    m_profiles = new SubsystemProfiles<>(periodicHash, ManipulatorState.kStow);

    m_wristIO.setPIDFF(
        ManipulatorConstants.kWristP.get(),
        ManipulatorConstants.kWristI.get(),
        ManipulatorConstants.kWristD.get(),
        ManipulatorConstants.kWristKS.get(),
        ManipulatorConstants.kWristKG.get());

    m_rollerIO.setPositionPID(
        ManipulatorConstants.kRollerP.get(),
        ManipulatorConstants.kRollerI.get(),
        ManipulatorConstants.kRollerD.get(),
        ManipulatorConstants.kRollerKS.get());
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_wristIO.setPIDFF(
              ManipulatorConstants.kWristP.get(),
              ManipulatorConstants.kWristI.get(),
              ManipulatorConstants.kWristD.get(),
              ManipulatorConstants.kWristKS.get(),
              ManipulatorConstants.kWristKG.get());
        },
        ManipulatorConstants.kWristP,
        ManipulatorConstants.kWristI,
        ManipulatorConstants.kWristD,
        ManipulatorConstants.kWristKS,
        ManipulatorConstants.kWristKG);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_rollerIO.setPositionPID(
              ManipulatorConstants.kRollerP.get(),
              ManipulatorConstants.kRollerI.get(),
              ManipulatorConstants.kRollerD.get(),
              ManipulatorConstants.kRollerKS.get());
        },
        ManipulatorConstants.kRollerP,
        ManipulatorConstants.kRollerI,
        ManipulatorConstants.kRollerD,
        ManipulatorConstants.kRollerKS);

    m_rollerIO.updateInputs(m_rollerInputs);
    m_wristIO.updateInputs(m_wristInputs);
    m_coralDetectorIO.updateInputs(m_coralDetectorInputs);

    if (FullTuningConstants.kFullTuningMode) {
      updateState(ManipulatorState.kFullTuning);
    }

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Manipulator/Roller", m_rollerInputs);
    Logger.processInputs("Manipulator/Wrist", m_wristInputs);
    Logger.processInputs("Manipulator/CoralDetector", m_coralDetectorInputs);
    Logger.recordOutput("Manipulator/State", m_profiles.getCurrentProfile());

    if (Constants.kUseAlerts && !m_rollerInputs.motorIsConnected) {
      m_rollerMotorDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    if (Constants.kUseAlerts && !m_wristInputs.motorIsConnected) {
      m_wristMotorDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    Logger.recordOutput("PeriodicTime/Manipulator", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(ManipulatorState state) {
    if (m_profiles.getCurrentProfile() == ManipulatorState.kFullTuning) {
      // if we are in full tuning mode we don't want to change the state
      return;
    }
    // TODO: this makes sense but i have the feeling it will cause an issue during testing
    // DON'T FORGET ABOUT THIS
    m_runRollerScoring = false;
    m_runRollerAlgaeDescoring = true;

    if (state == ManipulatorState.kIndexing) {
      // set it some amount forward
      Angle currPosition = m_rollerIO.getPosition();
      currPosition =
          currPosition.plus(Degrees.of(ManipulatorConstants.kRollerIndexingPosition.get()));
      m_rollerIO.setDesiredPosition(currPosition);
    }

    m_profiles.setCurrentProfile(state);
  }

  public ManipulatorState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredWristAngle(Rotation2d angle) {
    m_desiredWristAngle = angle;
  }

  public Rotation2d getDesiredWristAngle() {
    return m_desiredWristAngle;
  }

  public void stowPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.get());
  }

  public void intakingPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristIntakeAngle.get()));
    // if we have a game piece stop
    // if the indexer isn't running then the elevator and manipulator aren't both in position yet
    // so we don't want to intake yet
    if (m_coralDetectorIO.hasGamePiece()) {
      // m_rollerIO.setVoltage(0.0);
      updateState(ManipulatorState.kIndexing);
    } else if (RobotState.getInstance().getIndexerState() != IndexerState.kIndexing) {
      m_rollerIO.setVoltage(0.0);
    } else {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerIntakeVoltage.get());
    }
  }

  public void indexingPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristIntakeAngle.get()));
  }

  public void scoringPeriodic() {
    m_wristIO.setDesiredAngle(m_desiredWristAngle);
    if (m_runRollerScoring) {
      if (RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L1) {
        m_rollerIO.setVoltage(ManipulatorConstants.kRollerLowerScoringVoltage.get());
      } else {
        m_rollerIO.setVoltage(ManipulatorConstants.kRollerUpperScoringVoltage.get());
      }
    } else {
      m_rollerIO.setVoltage(0.0);
    }
  }

  public void algaeDescoringPeriodic() {
    m_wristIO.setDesiredAngle(
        Rotation2d.fromDegrees(ManipulatorConstants.kWristAlgaeDescoringAngle.get()));

    if (m_runRollerAlgaeDescoring) {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerAlgaeDescoringVoltage.get());
    } else {
      m_rollerIO.setVoltage(0.0);
    }
  }

  public void algaeHoldPeriodic() {
    m_wristIO.setDesiredAngle(
        Rotation2d.fromDegrees(ManipulatorConstants.kWristAlgaeHoldAngle.get()));
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerAlgaeHoldVoltage.get());
  }

  public void fullTuningPeriodic() {
    m_wristIO.setDesiredAngle(
        Rotation2d.fromDegrees(FullTuningConstants.kManipulatorWristSetpoint.get()));
    m_rollerIO.setVoltage(FullTuningConstants.kManipulatorRollerVoltage.get());
  }

  public void runRollerScoring() {
    m_runRollerScoring = true;
  }

  public void stopRollerScoring() {
    m_runRollerScoring = false;
  }

  public void runRollerAlgaeDescoring() {
    m_runRollerAlgaeDescoring = true;
  }

  public void stopRollerAlgaeDescoring() {
    m_runRollerAlgaeDescoring = false;
  }

  public void manageStowOrHold() {
    if (m_profiles.getCurrentProfile() == ManipulatorState.kAlgaeHold) {
      return;
    }

    updateState(ManipulatorState.kStow);
  }

  public boolean atSetpoint() {
    return m_wristIO.atSetpoint() || m_profiles.getCurrentProfile() == ManipulatorState.kAlgaeHold;
  }

  public boolean hasGamePiece() {
    return m_coralDetectorIO.hasGamePiece();
  }

  public Rotation2d getCurrAngle() {
    return m_wristIO.getCurrAngle();
  }

  public boolean rollerWithinTolerance() {
    return m_rollerIO.withinPositionTolerance();
  }
}
