package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Collections;
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
  private int m_maxMessagesLength = 0;

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

  /**
   * Update the current profile. This function will log the transition from the last profile to the
   * current profile.
   *
   * @param profile the new profile to set
   */
  public void setCurrentProfile(T profile) {
    m_lastProfile = m_currentProfile;
    m_currentProfile = profile;

    // TODO: check if this sacrifices performance with JMX
    m_currMessages.add(
        String.format(
            "%s to %s: %.3f",
            m_lastProfile.toString(), m_currentProfile.toString(), Timer.getFPGATimestamp()));
  }

  /**
   * Get the periodic function assocaiated with the current profile. If a periodic function is not
   * found, or if a null value is present in the map, the returned {@code Runnable} will print a
   * warning This function MUST be called every loop iteration, otherwise the logging will not work
   *
   * @return the periodic function associated with the current profile, guaranteed to be non-null
   */
  public Runnable getPeriodicFunction() {
    // logging
    if (m_currMessages.size() > 0) {
      // we fill the rest of the messages for formatting
      // if there are old values in advantagescope it looks weird
      // this was the best solution i came up with that didn't sacrifice much performance
      m_maxMessagesLength = Math.max(m_maxMessagesLength, m_currMessages.size());
      m_currMessages.addAll(Collections.nCopies(m_maxMessagesLength - m_currMessages.size(), ""));

      Logger.recordOutput(
          String.format("SubsystemProfiles/%s/Set", m_profileEnum.getSimpleName()),
          m_currMessages.toArray(String[]::new));

      m_currMessages.clear();
    }

    Runnable res = m_profilePeriodicFunctions.get(m_currentProfile);
    if (res == null) {
      res =
          () -> {
            System.out.println(
                String.format(
                    "WARNING: No periodic function for profile %s::%s",
                    m_profileEnum.getSimpleName(), m_currentProfile.toString()));
          };
    }
    return res;
  }

  /**
   * @return the current profile
   */
  public T getCurrentProfile() {
    return m_currentProfile;
  }

  /**
   * Reverts to the last profile that was previously set. If no profile has been set, this function
   * will do nothing.
   */
  public void revertToLastProfile() {
    setCurrentProfile(m_lastProfile);
  }
}
