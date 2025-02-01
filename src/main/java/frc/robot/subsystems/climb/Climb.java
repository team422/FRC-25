package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  private ClimbIO m_io;
  private final ClimbInputsAutoLogged m_inputs;

  public static enum ClimbState {
    kStow,
    kDeploy
  }

  private SubsystemProfiles<ClimbState> m_profiles;

  public Climb(ClimbIO io) {
    m_io = io;
    m_inputs = new ClimbInputsAutoLogged();

    m_io.setPID(
        ClimbConstants.kClimbP.get(), ClimbConstants.kClimbI.get(), ClimbConstants.kClimbD.get());

    // Create a map of periodic functions for each state, then make a SubsystemProfiles object
    Map<ClimbState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ClimbState.kStow, this::stowPeriodic);
    periodicHash.put(ClimbState.kDeploy, this::deployPeriodic);

    m_profiles = new SubsystemProfiles<Climb.ClimbState>(periodicHash, ClimbState.kStow);
  }

  @Override
  public void periodic() {

    double timeStartPeriodic = Timer.getFPGATimestamp();

    // update PIDFF Constants if any have changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_io.setPID(
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

    double dt = Timer.getFPGATimestamp() - timeStartPeriodic;

    Logger.recordOutput("Timing/climberPeriodic", dt);
  }

  public void updateState(ClimbState newState) {
    m_profiles.setCurrentProfile(newState);
  }

  public ClimbState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void stowPeriodic() {
    m_io.setDesiredAngle(Rotation2d.fromDegrees(ClimbConstants.kClimbStowPosRad.get()));
  }

  public void deployPeriodic() {
    m_io.setDesiredAngle(Rotation2d.fromDegrees(ClimbConstants.kClimbDeployPosRad.get()));
  }

  public void zeroEncoder() {
    m_io.zeroEncoder();
  }
}
