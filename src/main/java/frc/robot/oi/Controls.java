package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controls {
  public double getMovement();

  public double getRotation();

  public Trigger roller();
}
