package frc.robot.subsystems.intake;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.EqualsUtil;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.util.AlertManager;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeRollerIO m_rollerIO;
  private PivotIO m_pivotIO;

  private Alert m_rollerMotorDisconnectedAlert =
      new Alert("Intake Roller Motor Disconnected", AlertType.kError);
  private Alert m_pivotMotorDisconnectedAlert =
      new Alert("Intake Pivot Motor Disconnected", AlertType.kError);

  public final IntakeRollerInputsAutoLogged m_rollerInputs = new IntakeRollerInputsAutoLogged();
  public final PivotInputsAutoLogged m_pivotInputs = new PivotInputsAutoLogged();

  private Timer m_timer = new Timer();
  private Timer m_secondaryTimer = new Timer();

  private boolean m_readyToZero = false;

  public static enum IntakeState {
    kStow,
    kIntaking,
    kCoralHold,
    kCoralRunThrough,
    kOuttaking,
    kFunnelIntaking,
    kCoralEject,
    kFullTuning,
  }

  private SubsystemProfiles<IntakeState> m_profiles;

  public Intake(IntakeRollerIO rollerIO, PivotIO pivotIO) {
    m_rollerIO = rollerIO;
    m_pivotIO = pivotIO;

    Map<IntakeState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IntakeState.kStow, this::stowPeriodic);
    periodicHash.put(IntakeState.kIntaking, this::intakingPeriodic);
    periodicHash.put(IntakeState.kCoralHold, this::coralHoldPeriodic);
    periodicHash.put(IntakeState.kOuttaking, this::outtakingPeriodic);
    periodicHash.put(IntakeState.kFunnelIntaking, this::funnelIntakingPeriodic);
    periodicHash.put(IntakeState.kCoralRunThrough, this::coralRunThroughPeriodic);
    periodicHash.put(IntakeState.kCoralEject, this::coralEjectPeriodic);
    periodicHash.put(IntakeState.kFullTuning, this::fullTuningPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, IntakeState.kStow);

    // if sim use sim
    if (RobotBase.isReal()) {
      m_pivotIO.setPIDFF(
          0,
          IntakeConstants.kPivotP0.get(),
          IntakeConstants.kPivotI.get(),
          IntakeConstants.kPivotD0.get(),
          IntakeConstants.kPivotKS.get());
      m_pivotIO.setPIDFF(
          1,
          IntakeConstants.kPivotP1.get(),
          IntakeConstants.kPivotI.get(),
          IntakeConstants.kPivotD1.get(),
          IntakeConstants.kPivotKS.get());
    } else {
      m_pivotIO.setPIDFF(
          0, IntakeConstants.kPivotSimP, IntakeConstants.kPivotSimI, IntakeConstants.kPivotSimD, 0);
    }

    AlertManager.registerAlert(m_pivotMotorDisconnectedAlert, m_rollerMotorDisconnectedAlert);
  }

  public void updateState(IntakeState state) {
    if (m_profiles.getCurrentProfile() == IntakeState.kFullTuning) {
      // if we are in full tuning mode we don't want to change the state
      return;
    }

    m_timer.stop();
    m_timer.reset();

    m_secondaryTimer.stop();
    m_secondaryTimer.reset();

    m_profiles.setCurrentProfile(state);
  }

  public IntakeState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (RobotBase.isReal()) {
            m_pivotIO.setPIDFF(
                0,
                IntakeConstants.kPivotP0.get(),
                IntakeConstants.kPivotI.get(),
                IntakeConstants.kPivotD0.get(),
                IntakeConstants.kPivotKS.get());
            m_pivotIO.setPIDFF(
                1,
                IntakeConstants.kPivotP1.get(),
                IntakeConstants.kPivotI.get(),
                IntakeConstants.kPivotD1.get(),
                IntakeConstants.kPivotKS.get());
            m_pivotIO.setPIDFF(
                2,
                IntakeConstants.kPivotP2.get(),
                IntakeConstants.kPivotI.get(),
                IntakeConstants.kPivotD2.get(),
                IntakeConstants.kPivotKS.get());
          } else {
            m_pivotIO.setPIDFF(
                0,
                IntakeConstants.kPivotSimP,
                IntakeConstants.kPivotSimI,
                IntakeConstants.kPivotSimD,
                0.0);
          }
        },
        IntakeConstants.kPivotP0,
        IntakeConstants.kPivotP1,
        IntakeConstants.kPivotP2,
        IntakeConstants.kPivotI,
        IntakeConstants.kPivotD0,
        IntakeConstants.kPivotD1,
        IntakeConstants.kPivotD2,
        IntakeConstants.kPivotKS);

    m_rollerIO.updateInputs(m_rollerInputs);
    m_pivotIO.updateInputs(m_pivotInputs);

    if (DriverStation.isEnabled() && m_pivotInputs.velocityRPS > IntakeConstants.kZeroVelocityThreshold.get()) {
      m_readyToZero = true;
    }
    if (DriverStation.isEnabled()
        && m_readyToZero
        && m_pivotInputs.currAngleDeg > IntakeConstants.kZeroEncoderThreshold.get()
        && EqualsUtil.epsilonEquals(m_pivotInputs.velocityRPS, 0.0)) {
      m_pivotIO.zeroEncoder(IntakeConstants.kZeroEncoderValue);
      m_readyToZero = false;
      Logger.recordOutput("Intake/Zeroed", Timer.getTimestamp());
    }

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Intake/Roller", m_rollerInputs);
    Logger.processInputs("Intake/Pivot", m_pivotInputs);
    Logger.recordOutput("Intake/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("Intake/ReadyToZero", m_readyToZero);

    if (Constants.kUseAlerts && !m_rollerInputs.motorIsConnected) {
      m_rollerMotorDisconnectedAlert.set(true);
    } else {
      m_rollerMotorDisconnectedAlert.set(false);
    }

    if (Constants.kUseAlerts && !m_pivotInputs.motorIsConnected) {
      m_pivotMotorDisconnectedAlert.set(true);
    } else {
      m_pivotMotorDisconnectedAlert.set(false);
    }

    Logger.recordOutput("PeriodicTime/Intake", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void stowPeriodic() {
    m_rollerIO.setVoltage(IntakeConstants.kRollerStowVoltage.get());
    m_pivotIO.setSlot(0);
    if (RobotState.getInstance().getClimbState() != ClimbState.kMatch) {
      m_pivotIO.setDesiredAngle(
          Rotation2d.fromDegrees(IntakeConstants.kPivotClimbAngle.get()), 0.0);
    } else {
      m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotStowAngle.get()), 0.0);
    }
  }

  public void intakingPeriodic() {
    if (RobotState.getInstance().getOtbRunthrough()) {
      m_rollerIO.setVoltage(IntakeConstants.kRollerIntakeVoltage.get());
    } else {
      m_rollerIO.setVoltage(IntakeConstants.kRollerL1IntakeVoltage.get());
    }

    if (m_pivotInputs.currAngleDeg > IntakeConstants.kIntakingFeedforwardThreshold.get()) {
      m_pivotIO.setSlot(2);
      m_pivotIO.setDesiredAngle(
          Rotation2d.fromDegrees(IntakeConstants.kPivotIntakeAngle.get()),
          IntakeConstants.kIntakingFeedforward.get());
    } else {
      m_pivotIO.setSlot(1);
      m_pivotIO.setDesiredAngle(
          Rotation2d.fromDegrees(IntakeConstants.kPivotIntakeAngle.get()), 0.0);
    }
  }

  public void coralHoldPeriodic() {
    if (!m_timer.isRunning()) {
      m_timer.restart();
    }

    if (!m_timer.hasElapsed(IntakeConstants.kCoralRaiseTimeout.get())) {
      m_rollerIO.setVoltage(IntakeConstants.kRollerHoldVoltage.get());
      return;
    }

    m_rollerIO.setVoltage(IntakeConstants.kRollerHoldVoltage.get());
    m_pivotIO.setSlot(0);
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotHoldAngle.get()), 0.0);
  }

  public void outtakingPeriodic() {
    m_rollerIO.setVoltage(IntakeConstants.kRollerOuttakeVoltage.get());
    m_pivotIO.setSlot(0);
    m_pivotIO.setDesiredAngle(
        Rotation2d.fromDegrees(IntakeConstants.kPivotOuttakeAngle.get()), 0.0);
  }

  public void funnelIntakingPeriodic() {
    // when we have a game piece (update to indexing) the intake can come back
    if (RobotState.getInstance().getManipulatorState() == ManipulatorState.kIntaking) {
      m_pivotIO.setSlot(1);
      m_pivotIO.setDesiredAngle(
          Rotation2d.fromDegrees(IntakeConstants.kPivotFunnelIntakeAngle.get()), 0.0);
      m_rollerIO.setVoltage(IntakeConstants.kRollerFunnelIntakeVoltage.get());
    } else {
      m_pivotIO.setSlot(1);
      m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotStowAngle.get()), 0.0);
      m_rollerIO.setVoltage(IntakeConstants.kRollerStowVoltage.get());
    }
  }

  public void coralRunThroughPeriodic() {
    if (!m_timer.isRunning()) {
      m_timer.restart();
    }

    if (!m_timer.hasElapsed(IntakeConstants.kCoralRaiseTimeout.get())) {
      m_rollerIO.setVoltage(IntakeConstants.kRollerHoldVoltage.get());
      return;
    }

    m_pivotIO.setSlot(0);
    m_pivotIO.setDesiredAngle(
        Rotation2d.fromDegrees(IntakeConstants.kPivotCoralRunThroughAngle.get()), 0.0);
    // once we're in position we can run the roller
    if (atSetpoint()) {
      if (!m_secondaryTimer.isRunning()) {
        m_secondaryTimer.restart();
      }
      if (RobotState.getInstance().getIndexerState() != IndexerState.kIndexing) {
        updateState(IntakeState.kStow);
        return;
      }

      m_rollerIO.setVoltage(IntakeConstants.kRollerCoralRunThroughVoltage.get());
    } else {
      m_rollerIO.setVoltage(IntakeConstants.kRollerHoldVoltage.get());
    }
  }

  public void coralEjectPeriodic() {
    m_pivotIO.setSlot(0);
    m_pivotIO.setDesiredAngle(
        Rotation2d.fromDegrees(IntakeConstants.kPivotCoralRunThroughAngle.get()), 0.0);
    m_rollerIO.setVoltage(IntakeConstants.kRollerEjectVoltage.get());
  }

  public void fullTuningPeriodic() {
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotIntakeAngle.get()), 0.0);
  }

  public IntakeState getStowOrHold() {
    // if we are in the coral hold or run through state, we want to keep doing that
    // otherwise, stow
    if (m_profiles.getCurrentProfile() == IntakeState.kCoralHold
        || m_profiles.getCurrentProfile() == IntakeState.kCoralRunThrough) {
      return m_profiles.getCurrentProfile();
    }
    return IntakeState.kStow;
  }

  public IntakeState getIntakeOrOuttake() {
    // our button is a toggle, so we need to check if we are holding a game piece
    // if we are, outtake
    if (m_profiles.getCurrentProfile() == IntakeState.kCoralHold) {
      return IntakeState.kOuttaking;
    } else {
      return IntakeState.kIntaking;
    }
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(m_pivotInputs.currAngleDeg);
  }

  public boolean atSetpoint() {
    return Math.abs(m_pivotInputs.desiredAngleDeg - m_pivotInputs.currAngleDeg)
        < IntakeConstants.kPivotTolerance.get();
  }
}
