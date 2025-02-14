package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;

public class CtreBaseRefreshManager {
  // this is a static class and may not be instantiated
  private CtreBaseRefreshManager() {}

  private static List<StatusSignal<?>> m_signals = new ArrayList<>();

  /**
   * Updates all of the signals in the manager. This must be called BEFORE {@code
   * CommandScheduler.getInstance().run()} in the robot periodic to work properly.
   */
  public static void updateAll() {
    // convert to array for the varargs
    if (m_signals.isEmpty()) {
      return;
    }
    BaseStatusSignal.refreshAll(m_signals.stream().toArray(StatusSignal[]::new));
  }

  public static void addSignals(List<StatusSignal<?>> signals) {
    m_signals.addAll(signals);
  }

  public static void addSignal(StatusSignal<?> signal) {
    m_signals.add(signal);
  }
}
