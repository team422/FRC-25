package frc.robot.util;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * A utility class for managing subsystem profiles/states. This is a simple finite state machine
 * that allows you to manage profiles for a subsystem, based on an enum type. It also supports
 * periodic functions associated with each profile, which can be called every loop iteration.
 *
 * <p>onEnter and onExit callbacks are not supported.
 */
public class SubsystemProfiles<T extends Enum<T>> {
  private static boolean kUseLogging = true;
  private static boolean kWarningsEnabled = true;

  /**
   * Sets whether to use logging for the subsystem profiles. This will enable logging of profile
   * transitions and periodic function timings. If this is set to false, no logging will be done.
   *
   * @param useLogging whether to use logging for all subsystem profiles.
   */
  public static void setUseLogging(boolean useLogging) {
    kUseLogging = useLogging;
  }

  /**
   * Sets whether to enable warnings for missing periodic functions. If this is set to true, a
   * warning will be printed to the console if a periodic function is not found for the current
   * profile.
   *
   * @param warningsEnabled whether to enable warnings for missing periodic functions across all
   *     subsystem profiles.
   */
  public static void setWarningsEnabled(boolean warningsEnabled) {
    kWarningsEnabled = warningsEnabled;
  }

  private T m_currentProfile;
  private final Map<T, Runnable> m_profilePeriodicFunctions;
  private T m_lastProfile;
  private final Class<T> m_profileEnum;

  // for logging
  private final List<String> m_currMessages = new ArrayList<>();
  private int m_maxMessagesLength = 0;

  /**
   * Creates a new SubsystemProfiles instance.
   *
   * @param profilePeriodicFunctions a map of profiles to their periodic functions. The keys must be
   *     non-null.
   * @param defaultProfile the default profile to set initially. This must be non-null.
   */
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

    if (kUseLogging) {
      m_currMessages.add(
          String.format(
              "%s to %s: %.3f",
              m_lastProfile.toString(), m_currentProfile.toString(), Timer.getFPGATimestamp()));
    }
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
    if (kUseLogging && m_currMessages.size() > 0) {
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

    // if the profile is null we return a warning
    return m_profilePeriodicFunctions.getOrDefault(
        m_currentProfile,
        () -> {
          if (kWarningsEnabled) {
            System.out.println(
                String.format(
                    "WARNING: No periodic function for profile %s::%s",
                    m_profileEnum.getSimpleName(), m_currentProfile.toString()));
          }
        });
  }

  /**
   * Gets a timed version of the periodic function returned by {@code getPeriodicFunction}. This
   * will log the time taken to "PeriodicTime/EnumName", where EnumName is the name of the states
   * enum.
   *
   * <p>If {@code kUseLogging} is false, this will simply return the periodic function.
   *
   * @return the periodic function with a timing wrapper.
   */
  public Runnable getPeriodicFunctionTimed() {
    Runnable r = getPeriodicFunction();
    if (!kUseLogging) {
      return r;
    }

    return () -> {
      double start = HALUtil.getFPGATime();

      r.run();

      Logger.recordOutput(
          String.format("PeriodicTime/%s", m_profileEnum.getSimpleName()),
          (HALUtil.getFPGATime() - start) / 1000.0);
    };
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
