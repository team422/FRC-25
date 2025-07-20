package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.Constants.FullTuningConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIO;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorInputsAutoLogged;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIO;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.wrist.WristIO;
import frc.robot.subsystems.manipulator.wrist.WristInputsAutoLogged;
import frc.robot.util.AlertManager;
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
  private boolean m_runRollerBargeScoring = false;
  private boolean m_rollerPositionControlSet = false;

  private Timer m_timer = new Timer();
  private Timer m_secondaryTimer = new Timer();
  private boolean m_autoReversing = false;

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
    kAlgaeOuttake,
    kCoralEject,
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
    periodicHash.put(ManipulatorState.kAlgaeOuttake, this::algaeOuttakePeriodic);
    periodicHash.put(ManipulatorState.kCoralEject, this::coralEjectPeriodic);
    periodicHash.put(ManipulatorState.kFullTuning, this::fullTuningPeriodic);
    m_profiles = new SubsystemProfiles<>(periodicHash, ManipulatorState.kStow);
    if (RobotBase.isReal()) {
      m_wristIO.setPIDFF(
          ManipulatorConstants.kWristP.get(),
          ManipulatorConstants.kWristI.get(),
          ManipulatorConstants.kWristD.get(),
          ManipulatorConstants.kWristKS.get());

      m_rollerIO.setPositionPID(
          ManipulatorConstants.kRollerP.get(),
          ManipulatorConstants.kRollerI.get(),
          ManipulatorConstants.kRollerD.get(),
          ManipulatorConstants.kRollerKS.get());
    } else {
      m_wristIO.setPIDFF(
          ManipulatorConstants.kWristP.get(),
          ManipulatorConstants.kWristI.get(),
          ManipulatorConstants.kWristD.get(),
          ManipulatorConstants.kWristKS.get());

      m_rollerIO.setPositionPID(
          ManipulatorConstants.kSimRollerP,
          ManipulatorConstants.kSimRollerI,
          ManipulatorConstants.kSimRollerD,
          0.0);
    }

    AlertManager.registerAlert(m_wristMotorDisconnectedAlert, m_rollerMotorDisconnectedAlert);
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_wristIO.setPIDFF(
                ManipulatorConstants.kWristP.get(),
                ManipulatorConstants.kWristI.get(),
                ManipulatorConstants.kWristD.get(),
                ManipulatorConstants.kWristKS.get());
          } else {
            m_wristIO.setPIDFF(
                ManipulatorConstants.kSimWristP,
                ManipulatorConstants.kSimWristI,
                ManipulatorConstants.kSimWristD,
                0.0);
          }
        },
        ManipulatorConstants.kWristP,
        ManipulatorConstants.kWristI,
        ManipulatorConstants.kWristD,
        ManipulatorConstants.kWristKS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_rollerIO.setPositionPID(
                ManipulatorConstants.kRollerP.get(),
                ManipulatorConstants.kRollerI.get(),
                ManipulatorConstants.kRollerD.get(),
                ManipulatorConstants.kRollerKS.get());
          } else {
            m_rollerIO.setPositionPID(
                ManipulatorConstants.kSimRollerP,
                ManipulatorConstants.kSimRollerI,
                ManipulatorConstants.kSimRollerD,
                0.0);
          }
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

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Manipulator/Roller", m_rollerInputs);
    Logger.processInputs("Manipulator/Wrist", m_wristInputs);
    Logger.processInputs("Manipulator/CoralDetector", m_coralDetectorInputs);
    Logger.recordOutput("Manipulator/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("Manipulator/RollerAtSetpoint", rollerWithinTolerance());
    Logger.recordOutput("Manipulator/RollerError", getRollerError());
    Logger.recordOutput("Manipulator/RunRollerBargeScoring", m_runRollerBargeScoring);
    Logger.recordOutput("Manipulator/RunRollerAlgaeDescoring", m_runRollerBargeScoring);
    Logger.recordOutput("Manipulator/RunRollerScoring", m_runRollerScoring);

    if (Constants.kUseAlerts && !m_rollerInputs.motorIsConnected) {
      m_rollerMotorDisconnectedAlert.set(true);
    } else {
      m_rollerMotorDisconnectedAlert.set(false);
    }

    if (Constants.kUseAlerts && !m_wristInputs.motorIsConnected) {
      m_wristMotorDisconnectedAlert.set(true);
    } else {
      m_wristMotorDisconnectedAlert.set(false);
    }

    Logger.recordOutput("PeriodicTime/Manipulator", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(ManipulatorState state) {
    if (m_profiles.getCurrentProfile() == ManipulatorState.kFullTuning) {
      // if we are in full tuning mode we don't want to change the state
      return;
    }

    m_runRollerScoring = false;
    m_runRollerAlgaeDescoring = true;
    m_runRollerBargeScoring = false;
    m_rollerPositionControlSet = false;

    m_timer.stop();
    m_timer.reset();

    m_secondaryTimer.stop();
    m_secondaryTimer.reset();

    m_autoReversing = false;

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
    if (hasGamePiece()) {
      updateState(ManipulatorState.kIndexing);
    } else if (RobotState.getInstance().getIndexerState() != IndexerState.kIndexing) {
      m_rollerIO.setVoltage(0.0);
    } else if (m_secondaryTimer.isRunning()) {
      if (m_secondaryTimer.hasElapsed(0.25)) {
        m_rollerIO.setVoltage(ManipulatorConstants.kRollerIntakeVoltage.get());
        m_autoReversing = false;
      }
    } else {
      if (gamePieceInFunnel()) {
        if (!m_timer.isRunning()) {
          m_timer.restart();
        }
        if (m_timer.hasElapsed(DriverStation.isAutonomous() ? 1.0 : 0.5)) {
          m_autoReversing = true;
          if (!m_secondaryTimer.isRunning()) {
            m_secondaryTimer.start();
          }
          m_rollerIO.setVoltage(-ManipulatorConstants.kRollerIntakeVoltage.get());
        } else {
          m_rollerIO.setVoltage(ManipulatorConstants.kRollerIntakeVoltage.get());
        }
      } else {
        m_rollerIO.setVoltage(ManipulatorConstants.kRollerIntakeVoltage.get());
      }
    }
  }

  public void indexingPeriodic() {
    if (!fullyIndexed()) {
      m_rollerPositionControlSet = false;
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerIndexingVoltage.get());
    } else if (!m_rollerPositionControlSet) {
      m_rollerPositionControlSet = true;
      m_rollerIO.setDesiredPosition(
          getRollerPosition().plus(Degrees.of(ManipulatorConstants.kRollerIndexingPosition.get())));
    }
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristIntakeAngle.get()));
  }

  public void scoringPeriodic() {
    m_wristIO.setDesiredAngle(m_desiredWristAngle);
    if (m_runRollerScoring) {
      if (RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L1) {
        if (RobotState.getInstance().getCurrentAction() == RobotAction.kAutoScore) {
          m_rollerIO.setVoltage(ManipulatorConstants.kRollerL1ScoringVoltageAutoscore.get());
        } else {
          m_rollerIO.setVoltage(ManipulatorConstants.kRollerL1ScoringVoltageManual.get());
        }
      } else if (RobotState.getInstance().getDesiredReefHeight() == ReefHeight.L4) {
        m_rollerIO.setVoltage(ManipulatorConstants.kRollerL4ScoringVoltage.get());
      } else {
        m_rollerIO.setVoltage(ManipulatorConstants.kRollerL2L3ScoringVoltage.get());
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
    if (m_runRollerBargeScoring) {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerBargeVoltage.get());
    } else if (RobotState.getInstance().getCurrentAction() == RobotAction.kLollipopIntake) {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerAlgaeDescoringVoltage.get());
    } else {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerAlgaeHoldVoltage.get());
    }
  }

  public void algaeOuttakePeriodic() {
    m_wristIO.setDesiredAngle(
        Rotation2d.fromDegrees(ManipulatorConstants.kWristAlgaeOuttakeAngle.get()));
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerAlgaeOuttakeVoltage.get());
  }

  public void coralEjectPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristIntakeAngle.get()));
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerEjectVoltage.get());
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

  public void runRollerBargeScoring() {
    m_runRollerBargeScoring = true;
  }

  public void stopRollerBargeScoring() {
    m_runRollerBargeScoring = false;
  }

  public ManipulatorState getStowOrHold() {
    if (m_profiles.getCurrentProfile() == ManipulatorState.kAlgaeHold) {
      return ManipulatorState.kAlgaeHold;
    }

    return ManipulatorState.kStow;
  }

  public boolean atSetpoint() {
    return Math.abs(m_wristInputs.desiredAngleDeg - getCurrAngle().getDegrees())
            < ManipulatorConstants.kWristTolerance
        || m_profiles.getCurrentProfile() == ManipulatorState.kAlgaeHold;
  }

  public boolean atSetpoint(Rotation2d tolerance) {
    return Math.abs(tolerance.getDegrees() - getCurrAngle().getDegrees())
        < ManipulatorConstants.kWristTolerance;
  }

  public boolean hasGamePiece() {
    return m_coralDetectorInputs.hasGamePiece;
  }

  public boolean fullyIndexed() {
    return hasGamePiece() && !gamePieceInFunnel();
  }

  public boolean gamePieceInFunnel() {
    return m_coralDetectorInputs.gamePieceInFunnel;
  }

  public Rotation2d getCurrAngle() {
    return Rotation2d.fromDegrees(m_wristInputs.currAngleDeg);
  }

  public Angle getRollerPosition() {
    return Degrees.of(m_rollerInputs.positionDegrees);
  }

  public boolean rollerWithinTolerance() {
    return m_rollerInputs.positionControl
        && Math.abs(m_rollerInputs.positionDegrees - m_rollerInputs.desiredPositionDegrees)
            < ManipulatorConstants.kRollerPositionTolerance;
  }

  public double getRollerError() {
    return m_rollerInputs.desiredPositionDegrees - m_rollerInputs.positionDegrees;
  }

  public boolean getAutoReversing() {
    return m_autoReversing;
  }
}
