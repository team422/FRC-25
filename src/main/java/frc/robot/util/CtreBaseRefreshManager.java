package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;

public class CtreBaseRefreshManager {
  // create a singleton
  private static CtreBaseRefreshManager instance = null;

  // create a arraylist with all of the status signals
  private static List<StatusSignal<?>> ctreBaseRefreshList = new ArrayList<>();
  // make a regular list that is not an arraylist

  public static CtreBaseRefreshManager getInstance() {
    if (instance == null) {
      instance = new CtreBaseRefreshManager();
    }
    return instance;
  }

  private CtreBaseRefreshManager() {}

  public void updateAll() {
    // use BaseSignal to update all of the signals
    BaseStatusSignal.refreshAll(ctreBaseRefreshList.stream().toArray(StatusSignal[]::new));
  }

  public void addSignals(List<StatusSignal<?>> signals) {
    ctreBaseRefreshList.addAll(signals);
  }

  public void addSignal(StatusSignal<?> signal) {
    ctreBaseRefreshList.add(signal);
  }
}
