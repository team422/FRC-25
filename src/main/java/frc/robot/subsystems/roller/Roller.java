package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private RollerIO m_io;
  private SubsystemProfiles<RollerProfiles> m_profiles;
  public final RollerInputsAutoLogged m_rollerInputs;

  public enum RollerProfiles {
    kIdle,
    kOuttake,
  }

  public Roller(RollerIO io) {
    m_io = io;
    m_rollerInputs = new RollerInputsAutoLogged();
    HashMap<RollerProfiles, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(RollerProfiles.kIdle, this::idlePeriodic);
    periodicHash.put(RollerProfiles.kOuttake, this::outtakePeriodic);
    m_profiles = new SubsystemProfiles<>(periodicHash, RollerProfiles.kIdle);
  }

  public void periodic() {
    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Roller/Inputs", m_rollerInputs);
    Logger.recordOutput("Roller/State", m_profiles.getCurrentProfile());

    m_io.updateInputs(m_rollerInputs);
  }

  public void updateInputs() {
    m_io.updateInputs(m_rollerInputs);
  }

  public void idlePeriodic() {
    m_io.setVoltage(0);
  }

  public void outtakePeriodic() {
    m_io.setVoltage(Constants.RollerConstants.kRollerSpeed);
  }

  public void updateState(RollerProfiles state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(0);
        break;
      case kOuttake:
        m_io.setVoltage(Constants.RollerConstants.kRollerSpeed);
        break;
    }
  }

  public RollerProfiles getState() {
    return (RollerProfiles) m_profiles.getCurrentProfile();
  }
}
