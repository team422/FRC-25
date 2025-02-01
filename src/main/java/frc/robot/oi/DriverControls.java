package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public Trigger level1();

  public Trigger level2();

  public Trigger level3();

  public Trigger level4();

  public Trigger runIndexer();

  public Trigger intake();

  public Trigger manipulate();

  public Trigger stowElevator();
}
