package frc.robot.subsystems.climb;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.RobotState;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIO m_io;
  private final ClimbInputsAutoLogged m_inputs;

  private Alert m_motorDisconnectedAlert = new Alert("Climb Motor Disconnected", AlertType.kError);

  public static enum ClimbState {
    kStow,
    kMatch,
    kDeploy,
  }

  private SubsystemProfiles<ClimbState> m_profiles;

  private int m_currSlot = 0;

  public Climb(ClimbIO io) {
    m_io = io;
    m_inputs = new ClimbInputsAutoLogged();

    m_io.setPID(
        m_currSlot,
        ClimbConstants.kClimbP.get(),
        ClimbConstants.kClimbI.get(),
        ClimbConstants.kClimbD.get());

    // Create a map of periodic functions for each state, then make a SubsystemProfiles object
    Map<ClimbState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ClimbState.kStow, this::stowPeriodic);
    periodicHash.put(ClimbState.kMatch, this::matchPeriodic);
    periodicHash.put(ClimbState.kDeploy, this::deployPeriodic);

    m_profiles = new SubsystemProfiles<Climb.ClimbState>(periodicHash, ClimbState.kStow);

    zeroEncoder();
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    // update PIDFF Constants if any have changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_io.setPID(
              m_currSlot,
              ClimbConstants.kClimbP.get(),
              ClimbConstants.kClimbI.get(),
              ClimbConstants.kClimbD.get());
        },
        ClimbConstants.kClimbP,
        ClimbConstants.kClimbI,
        ClimbConstants.kClimbD);

    // update inputs and run periodic function for current state
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunction().run();

    // log inputs and profile
    Logger.processInputs("Climb", m_inputs);
    Logger.recordOutput("Climb/State", m_profiles.getCurrentProfile());

    if (Constants.kUseAlerts && !m_inputs.motorIsConnected) {
      m_motorDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    Logger.recordOutput("PeriodicTime/Climb", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(ClimbState newState) {
    m_profiles.setCurrentProfile(newState);
  }

  public ClimbState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void stowPeriodic() {
    m_io.setSlot(0);
    m_io.setDesiredAngle(
        Rotation2d.fromDegrees(ClimbConstants.kClimbStowPos.get()),
        ClimbConstants.kStowFeedforward.get());
  }

  public void matchPeriodic() {
    m_io.setSlot(0);
    m_io.setDesiredAngle(Rotation2d.fromDegrees(ClimbConstants.kClimbMatchPos.get()), 0.0);
  }

  public void deployPeriodic() {
    m_io.setSlot(0);
    m_io.setDesiredAngle(Rotation2d.fromDegrees(ClimbConstants.kClimbDeployPos.get()), 0.0);
  }

  public void zeroEncoder() {
    m_io.zeroEncoder();
  }
}
