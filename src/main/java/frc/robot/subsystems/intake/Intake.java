package frc.robot.subsystems.intake;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
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

  public static enum IntakeState {
    kStow,
    kIntake,
    kGamepieceHold,
    kOuttake,
    kCoralIntaking,
    kFullTuning,
  }

  private SubsystemProfiles<IntakeState> m_profiles;

  public Intake(IntakeRollerIO rollerIO, PivotIO pivotIO) {
    m_rollerIO = rollerIO;
    m_pivotIO = pivotIO;

    Map<IntakeState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IntakeState.kStow, this::stowPeriodic);
    periodicHash.put(IntakeState.kIntake, this::intakePeriodic);
    periodicHash.put(IntakeState.kGamepieceHold, this::gamepieceHoldPeriodic);
    periodicHash.put(IntakeState.kOuttake, this::outtakePeriodic);
    periodicHash.put(IntakeState.kCoralIntaking, this::coralIntakingPeriodic);
    periodicHash.put(IntakeState.kFullTuning, this::fullTuningPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, IntakeState.kStow);

    m_pivotIO.setPIDFF(
        IntakeConstants.kPivotP.get(),
        IntakeConstants.kPivotI.get(),
        IntakeConstants.kPivotD.get(),
        IntakeConstants.kPivotKS.get(),
        IntakeConstants.kPivotKG.get());
  }

  public void updateState(IntakeState state) {
    if (m_profiles.getCurrentProfile() == IntakeState.kFullTuning) {
      // if we are in full tuning mode we don't want to change the state
      return;
    }

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
          m_pivotIO.setPIDFF(
              IntakeConstants.kPivotP.get(),
              IntakeConstants.kPivotI.get(),
              IntakeConstants.kPivotD.get(),
              IntakeConstants.kPivotKS.get(),
              IntakeConstants.kPivotKG.get());
        },
        IntakeConstants.kPivotP,
        IntakeConstants.kPivotI,
        IntakeConstants.kPivotD,
        IntakeConstants.kPivotKS,
        IntakeConstants.kPivotKG);

    m_rollerIO.updateInputs(m_rollerInputs);
    m_pivotIO.updateInputs(m_pivotInputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Intake/Roller", m_rollerInputs);
    Logger.processInputs("Intake/Pivot", m_pivotInputs);
    Logger.recordOutput("Intake/State", m_profiles.getCurrentProfile());

    if (Constants.kUseAlerts && !m_rollerInputs.motorIsConnected) {
      m_rollerMotorDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    if (Constants.kUseAlerts && !m_pivotInputs.motorIsConnected) {
      m_pivotMotorDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    Logger.recordOutput("PeriodicTime/Intake", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void stowPeriodic() {
    m_rollerIO.setVoltage(IntakeConstants.kRollerStowVoltage.get());
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotStowAngle.get()));
  }

  public void intakePeriodic() {
    m_rollerIO.setVoltage(IntakeConstants.kRollerIntakeVoltage.get());
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotIntakeAngle.get()));
  }

  public void gamepieceHoldPeriodic() {
    m_rollerIO.setVoltage(IntakeConstants.kRollerHoldVoltage.get());
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotHoldAngle.get()));
  }

  public void outtakePeriodic() {
    m_rollerIO.setVoltage(IntakeConstants.kRollerOuttakeVoltage.get());
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotOuttakeAngle.get()));
  }

  public void coralIntakingPeriodic() {
    // when we have a game piece (update to indexing) the intake can come back
    if (RobotState.getInstance().getManipulatorState() == ManipulatorState.kIntaking) {
      m_pivotIO.setDesiredAngle(
          Rotation2d.fromDegrees(IntakeConstants.kPivotCoralIntakeAngle.get()));
      m_rollerIO.setVoltage(IntakeConstants.kRollerIntakeVoltage.get());
    } else {
      m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotStowAngle.get()));
      m_rollerIO.setVoltage(IntakeConstants.kRollerStowVoltage.get());
    }
  }

  public void fullTuningPeriodic() {
    m_pivotIO.setDesiredAngle(Rotation2d.fromDegrees(IntakeConstants.kPivotIntakeAngle.get()));
  }

  public IntakeState getStowOrHold() {
    // if we have a game piece, do nothing and continue to hold
    // otherwise, stow
    if (m_profiles.getCurrentProfile() == IntakeState.kGamepieceHold) {
      return IntakeState.kGamepieceHold;
    }
    return IntakeState.kStow;
  }

  public IntakeState getIntakeOrOuttake() {
    // our button is a toggle, so we need to check if we are holding a game piece
    // if we are, outtake
    if (m_profiles.getCurrentProfile() == IntakeState.kGamepieceHold) {
      return IntakeState.kOuttake;
    } else {
      return IntakeState.kIntake;
    }
  }

  public boolean hasGamePiece() {
    return m_rollerIO.hasGamePiece();
  }

  public Rotation2d getRotation() {
    return m_pivotIO.getCurrAngle();
  }
}
