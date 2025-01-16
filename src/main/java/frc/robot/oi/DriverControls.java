package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public Trigger flywheelIdle();

  public Trigger flywheelSetpoint1();

  public Trigger flywheelSetpoint2();

  public Trigger flywheelEject();
}
