package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.hal.HALUtil;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// Tommy - this class should not be used because it does not work
// feel free to debug it if you have time, but it is not a priority
public class CtreBaseRefreshManager {
  // this is a static class and may not be instantiated
  private CtreBaseRefreshManager() {}

  private static List<StatusSignal<?>> m_signals = new ArrayList<>();

  /**
   * Updates all of the signals in the manager. This must be called BEFORE {@code
   * CommandScheduler.getInstance().run()} in the robot periodic to work properly.
   */
  public static void updateAll() {
    double start = HALUtil.getFPGATime();
    // convert to array for the varargs
    if (m_signals.isEmpty()) {
      return;
    }
    var status = BaseStatusSignal.refreshAll(m_signals.stream().toArray(StatusSignal[]::new));
    Logger.recordOutput("CtreBaseRefreshManager/StatusCode", status);
    Logger.recordOutput("CtreBaseRefreshManager/RegisteredSignals", m_signals.size());
    Logger.recordOutput(
        "PeriodicTime/CtreBaseRefreshManager", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public static void addSignals(List<StatusSignal<?>> signals) {
    m_signals.addAll(signals);
  }

  public static void addSignal(StatusSignal<?> signal) {
    m_signals.add(signal);
  }
}
