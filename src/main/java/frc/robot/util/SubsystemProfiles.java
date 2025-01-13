package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class SubsystemProfiles<T extends Enum<T>> {
  private T m_currentProfile;
  private Map<T, Runnable> m_profilePeriodicFunctions;
  private T m_lastProfile;
  private Class<T> m_profileEnum;

  // for logging
  private List<String> m_currMessages = new ArrayList<>();
  private double m_currTimestamp = -1;

  @SuppressWarnings("unchecked")
  public SubsystemProfiles(Map<T, Runnable> profilePeriodicFunctions, T defaultProfile) {

    // no null profiles allowed
    if (defaultProfile == null) {
      throw new IllegalArgumentException("Default profile cannot be null");
    }

    // we can ignore the unchecked warning because we know that the class is of type T
    m_profileEnum = (Class<T>) defaultProfile.getClass();
    m_currentProfile = defaultProfile;
    m_profilePeriodicFunctions = profilePeriodicFunctions;
    m_lastProfile = m_currentProfile;
  }

  public void setCurrentProfile(T profile) {
    m_lastProfile = m_currentProfile;
    m_currentProfile = profile;
    if (m_currTimestamp == Timer.getFPGATimestamp()) {
      m_currMessages.add(
          String.format(
              "%s to %s: %:.3f",
              m_lastProfile.toString(), m_currentProfile.toString(), Timer.getFPGATimestamp()));
    } else {
      Logger.recordOutput(
          String.format("SubsystemProfiles/%s/Set", m_profileEnum.getSimpleName()),
          m_currMessages.toArray(String[]::new));
      m_currMessages.clear();
      m_currTimestamp = Timer.getFPGATimestamp();
    }
  }

  public Runnable getPeriodicFunction() {
    return m_profilePeriodicFunctions.getOrDefault(
        m_currentProfile,
        () -> {
          System.out.println(
              String.format(
                  "WARNING: No periodic function for profile %s::%s",
                  m_profileEnum.getSimpleName(), m_currentProfile.toString()));
        });
  }

  public T getCurrentProfile() {
    return m_currentProfile;
  }

  public void revertToLastProfile() {
    setCurrentProfile(m_lastProfile);
  }
}
