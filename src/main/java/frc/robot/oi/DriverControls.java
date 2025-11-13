package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public Trigger setL1();

  public Trigger setL2();

  public Trigger setL3();

  public Trigger setL4();

  public Trigger manualScore();
}
