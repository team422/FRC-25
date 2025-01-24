package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public default Trigger indexerIdle() {
    return new Trigger(() -> false);
  }

  public default Trigger indexerIndexing() {
    return new Trigger(() -> false);
  }
}
