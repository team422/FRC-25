package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AlertManager {
  private static List<Alert> m_alerts = new ArrayList<>();

  // this is a static class and cannot be instantiated without reflection
  private AlertManager() {}

  public static void registerAlert(Alert... alerts) {
    m_alerts.addAll(Arrays.asList(alerts));
  }

  /**
   * Updates all of the alerts in the manager. This must be called AFTER {@code
   * CommandScheduler.getInstance().run()} in the robot periodic to work properly.
   */
  public static void update() {
    // boolean anyActive = false;
    // for (Alert alert : m_alerts) {
    //   anyActive |= alert.get();
    // }

    // if (anyActive) {
    //   RobotState.getInstance().triggerAlert();
    // } else {
    //   RobotState.getInstance().cancelAlert();
    // }
  }
}
