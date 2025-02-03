package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotInputsAutoLogged;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeRollerIO m_rollerIO;
  private PivotIO m_pivotIO;

  public final IntakeRollerInputsAutoLogged m_rollerInputs = new IntakeRollerInputsAutoLogged();
  public final PivotInputsAutoLogged m_pivotInputs = new PivotInputsAutoLogged();

  public static enum IntakeState {
    kStow,
    kIntake,
    kGamepieceHold,
    kOuttake,
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

    m_profiles = new SubsystemProfiles<>(periodicHash, IntakeState.kStow);
  }

  public void updateState(IntakeState state) {
    m_profiles.setCurrentProfile(state);
  }

  public IntakeState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  @Override
  public void periodic() {
    double start = Timer.getFPGATimestamp();

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
    Logger.recordOutput("PeriodicTime/Intake", Timer.getFPGATimestamp() - start);
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

  public void manageStowOrHold() {
    // if we have a game piece, do nothing and continue to hold
    // otherwise, stow
    if (m_profiles.getCurrentProfile() == IntakeState.kGamepieceHold) {
      return;
    }
    updateState(IntakeState.kStow);
  }

  public void manageIntakeOrOuttake() {
    // our button is a toggle, so we need to check if we are holding a game piece
    // if we are, outtake
    if (m_profiles.getCurrentProfile() == IntakeState.kGamepieceHold) {
      updateState(IntakeState.kOuttake);
    } else {
      updateState(IntakeState.kIntake);
    }
  }

  public boolean hasGamePiece() {
    return m_rollerIO.hasGamePiece();
  }
}
