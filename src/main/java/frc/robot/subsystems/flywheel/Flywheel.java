package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private FlywheelIO m_io;
  public final FlywheelInputsAutoLogged m_inputs;

  public static enum FlywheelState {
    kIdle,
    kRevving,
    kEjecting,
  }

  private SubsystemProfiles<FlywheelState> m_profiles;

  private double m_desiredVelocity;

  public Flywheel(FlywheelIO io) {
    m_io = io;
    m_inputs = new FlywheelInputsAutoLogged();

    m_io.setPIDFF(
        FlywheelConstants.kFlywheelP.get(),
        FlywheelConstants.kFlywheelI.get(),
        FlywheelConstants.kFlywheelD.get(),
        FlywheelConstants.kFlywheelKS.get(),
        FlywheelConstants.kFlywheelKV.get());

    Map<FlywheelState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(FlywheelState.kIdle, this::idlePeriodic);
    periodicHash.put(FlywheelState.kRevving, this::revvingPeriodic);
    periodicHash.put(FlywheelState.kEjecting, this::ejectingPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, FlywheelState.kIdle);
  }

  public void updateState(FlywheelState state) {
    m_profiles.setCurrentProfile(state);
  }

  public FlywheelState getState() {
    return m_profiles.getCurrentProfile();
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_io.setPIDFF(
              FlywheelConstants.kFlywheelP.get(),
              FlywheelConstants.kFlywheelI.get(),
              FlywheelConstants.kFlywheelD.get(),
              FlywheelConstants.kFlywheelKS.get(),
              FlywheelConstants.kFlywheelKV.get());
        },
        FlywheelConstants.kFlywheelP,
        FlywheelConstants.kFlywheelI,
        FlywheelConstants.kFlywheelD,
        FlywheelConstants.kFlywheelKS,
        FlywheelConstants.kFlywheelKV);

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Flywheel", m_inputs);
    Logger.recordOutput("Flywheel/State", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(0);
  }

  public void revvingPeriodic() {
    m_io.setDesiredVelocity(m_desiredVelocity);
  }

  public void ejectingPeriodic() {
    m_io.setVoltage(-7);
  }

  public void setDesiredVelocity(double velocityRPS) {
    m_desiredVelocity = velocityRPS;
  }
}
