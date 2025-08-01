package frc.robot.subsystems.elevator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.Constants.FullTuningConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.util.AlertManager;
import frc.robot.util.SetpointGenerator;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public final ElevatorInputsAutoLogged m_inputs;
  private ElevatorIO m_io;
  private double m_desiredHeight;
  private Timer m_timer = new Timer();

  private Alert m_leadingMotorDisconnectedAlert =
      new Alert("Elevator Leading Motor Disconnected", AlertType.kError);
  private Alert m_followingMotorDisconnectedAlert =
      new Alert("Elevator Following Motor Disconnected", AlertType.kError);

  public static enum ElevatorState {
    kStow,
    kScoring,
    kIntaking,
    kAlgaeDescoringInitial,
    kAlgaeDescoringFinal,
    kBargeScore,
    kLollipopIntake,
    kFullTuning,
    kSlamming,
    // we force a slam every time we go down
  }

  private SubsystemProfiles<ElevatorState> m_profiles;

  public Elevator(ElevatorIO io) {
    m_io = io;
    m_inputs = new ElevatorInputsAutoLogged();

    Map<ElevatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ElevatorState.kStow, this::stowPeriodic);
    periodicHash.put(ElevatorState.kScoring, this::scoringPeriodic);
    periodicHash.put(ElevatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ElevatorState.kAlgaeDescoringInitial, this::algaeDescoringInitialPeriodic);
    periodicHash.put(ElevatorState.kAlgaeDescoringFinal, this::algaeDescoringFinalPeriodic);
    periodicHash.put(ElevatorState.kBargeScore, this::bargeScorePeriodic);
    periodicHash.put(ElevatorState.kLollipopIntake, this::lollipopIntakePeriodic);
    periodicHash.put(ElevatorState.kFullTuning, this::fullTuningPeriodic);
    periodicHash.put(ElevatorState.kSlamming, this::slammingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, ElevatorState.kStow);

    if (RobotBase.isReal()) {
      m_io.setPIDFF(
          0,
          ElevatorConstants.kP0.getAsDouble(),
          ElevatorConstants.kI.getAsDouble(),
          ElevatorConstants.kD.getAsDouble(),
          ElevatorConstants.kKG0.getAsDouble());

      m_io.setPIDFF(
          1,
          ElevatorConstants.kP1.getAsDouble(),
          ElevatorConstants.kI.getAsDouble(),
          ElevatorConstants.kD.getAsDouble(),
          ElevatorConstants.kKG1.getAsDouble());

      m_io.setPIDFF(
          2,
          ElevatorConstants.kP2.getAsDouble(),
          ElevatorConstants.kI.getAsDouble(),
          ElevatorConstants.kD.getAsDouble(),
          ElevatorConstants.kKG2.getAsDouble());
    } else {
      m_io.setPIDFF(
          0,
          ElevatorConstants.kSimElevatorP,
          ElevatorConstants.kSimElevatorI,
          ElevatorConstants.kSimElevatorD,
          ElevatorConstants.kSimElevatorkG);
    }

    // to set the setpoint
    updateState(ElevatorState.kStow);

    AlertManager.registerAlert(m_leadingMotorDisconnectedAlert, m_followingMotorDisconnectedAlert);
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
          if (RobotBase.isReal()) {
            m_io.setPIDFF(
                0,
                ElevatorConstants.kP0.getAsDouble(),
                ElevatorConstants.kI.getAsDouble(),
                ElevatorConstants.kD.getAsDouble(),
                ElevatorConstants.kKG0.getAsDouble());

            m_io.setPIDFF(
                1,
                ElevatorConstants.kP1.getAsDouble(),
                ElevatorConstants.kI.getAsDouble(),
                ElevatorConstants.kD.getAsDouble(),
                ElevatorConstants.kKG1.getAsDouble());

            m_io.setPIDFF(
                2,
                ElevatorConstants.kP2.getAsDouble(),
                ElevatorConstants.kI.getAsDouble(),
                ElevatorConstants.kD.getAsDouble(),
                ElevatorConstants.kKG2.getAsDouble());
          } else {
            m_io.setPIDFF(
                0,
                ElevatorConstants.kSimElevatorP,
                ElevatorConstants.kSimElevatorI,
                ElevatorConstants.kSimElevatorD,
                ElevatorConstants.kSimElevatorkG);
          }
        },
        ElevatorConstants.kP0,
        ElevatorConstants.kP1,
        ElevatorConstants.kP2,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.kKG0,
        ElevatorConstants.kKG1,
        ElevatorConstants.kKG2);

    m_io.updateInputs(m_inputs);

    double currHeight = m_inputs.leadingPosition;
    if (currHeight < 18) {
      m_io.setSlot(0);
      m_io.setDesiredHeight(m_inputs.desiredLocation);
    } else if (currHeight < 45) {
      m_io.setSlot(1);
      m_io.setDesiredHeight(m_inputs.desiredLocation);
    } else {
      m_io.setSlot(2);
      m_io.setDesiredHeight(m_inputs.desiredLocation);
    }

    if (shouldSlamElevator()) {
      updateState(ElevatorState.kSlamming);
    }

    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Elevator", m_inputs);
    Logger.recordOutput("Elevator/State", m_profiles.getCurrentProfile());

    if (shouldZeroElevator()) {
      m_io.zeroElevator();
    }

    if (Constants.kUseAlerts && !m_inputs.isLeadingMotorConnected) {
      m_leadingMotorDisconnectedAlert.set(true);
    } else {
      m_leadingMotorDisconnectedAlert.set(false);
    }

    if (Constants.kUseAlerts && !m_inputs.isFollowingMotorConnected) {
      m_followingMotorDisconnectedAlert.set(true);
    } else {
      m_followingMotorDisconnectedAlert.set(false);
    }

    Logger.recordOutput("PeriodicTime/Elevator", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(ElevatorState state) {
    if (m_profiles.getCurrentProfile() == ElevatorState.kFullTuning) {
      // if we are in full tuning mode we don't want to change the state
      return;
    }
    if (state == ElevatorState.kSlamming) {
      m_timer.restart();
    }

    m_profiles.setCurrentProfile(state);
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

  public boolean shouldZeroElevator() {
    return m_profiles.getCurrentProfile() != ElevatorState.kSlamming
        && Math.abs(m_inputs.leadingVelocity) < ElevatorConstants.kZeroVelocityThreshold
        && m_inputs.leadingSupplyCurrent > ElevatorConstants.kZeroCurrentTheshold;
  }

  public boolean shouldSlamElevator() {
    // I AM JAMES BAE AND I WROTE THIS
    // if the position is between tolerance and max skip amount we cut power so it slams and hard
    // resets only when velocity is going down

    if (m_profiles.getCurrentProfile() != ElevatorState.kSlamming
        && m_inputs.leadingPosition > ElevatorConstants.kHeightTolerance
        && m_inputs.leadingPosition < ElevatorConstants.kMaxSkip
        && m_inputs.leadingVelocity < 0) {
      return true;
    }
    return false;
  }

  public double getDesiredHeight() {
    return m_desiredHeight;
  }

  public double getCurrHeight() {
    return m_inputs.leadingPosition;
  }

  public void stowPeriodic() {
    // wait for the manipulator to move out of the way before descending
    if (RobotState.getInstance().manipulatorAtSetpoint()) {
      if (RobotState.getInstance().getManipulatorState() == ManipulatorState.kAlgaeHold) {
        m_io.setDesiredHeight(ElevatorConstants.kAlgaeHoldHeight.get());
      } else if (RobotState.getInstance().getManipulatorState() == ManipulatorState.kAlgaeOuttake) {
        m_io.setDesiredHeight(ElevatorConstants.kAlgaeOuttakeHeight.get());
      } else {
        m_io.setDesiredHeight(ElevatorConstants.kStowHeight.get());
      }
    }
  }

  public void lollipopIntakePeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kLollipopIntakeHeight.get());
  }

  public void scoringPeriodic() {
    m_io.setDesiredHeight(m_desiredHeight);
  }

  public void intakingPeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kIntakingHeight.get());
  }

  public void algaeDescoringInitialPeriodic() {
    ReefHeight currHeight;
    if (RobotState.getInstance().getUsingVision()) {
      currHeight = SetpointGenerator.getAlgaeHeight(RobotState.getInstance().getRobotPose());
    } else {
      currHeight = RobotState.getInstance().getDesiredReefHeight();
    }
    if (currHeight == ReefHeight.L2) {
      m_io.setDesiredHeight(ElevatorConstants.kAlgaeDescoringIntialHeightL2.get());
    } else {
      m_io.setDesiredHeight(ElevatorConstants.kAlgaeDescoringIntialHeightL3.get());
    }
  }

  public void algaeDescoringFinalPeriodic() {
    ReefHeight currHeight;
    if (RobotState.getInstance().getUsingVision()) {
      currHeight = SetpointGenerator.getAlgaeHeight(RobotState.getInstance().getRobotPose());
    } else {
      currHeight = RobotState.getInstance().getDesiredReefHeight();
    }
    if (currHeight == ReefHeight.L2) {
      m_io.setDesiredHeight(ElevatorConstants.kAlgaeDescoringFinalHeightL2.get());
    } else {
      m_io.setDesiredHeight(ElevatorConstants.kAlgaeDescoringFinalHeightL3.get());
    }
  }

  public void bargeScorePeriodic() {
    m_io.setDesiredHeight(ElevatorConstants.kBargeScoreHeight.get());
  }

  public void fullTuningPeriodic() {
    m_io.setDesiredHeight(FullTuningConstants.kElevatorSetpoint.get());
  }

  public void slammingPeriodic() {
    m_io.setVoltage(ElevatorConstants.kSlamVoltage.get());
    if (m_timer.hasElapsed(ElevatorConstants.kSlamTime.get())) {
      m_io.zeroElevator();
      m_profiles.revertToLastProfile();
      // this is KRAZY we're using revert to last profile
    }
  }

  public boolean atSetpoint() {
    return Math.abs(m_inputs.leadingPosition - m_inputs.desiredLocation)
        <= ElevatorConstants.kHeightTolerance;
  }

  public boolean atSetpoint(double tolerance) {
    return Math.abs(m_inputs.leadingPosition - m_inputs.desiredLocation) <= tolerance;
  }

  public void zeroElevator() {
    m_io.zeroElevator();
  }
}
