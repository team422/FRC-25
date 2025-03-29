package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getForward();

  public double getStrafe();

  public double getTurn();

  public Trigger resetFieldCentric();

  public Trigger coralIntake();

  public Trigger coralOuttake();

  public Trigger setLocationL1();

  public Trigger setLocationL2();

  public Trigger setLocationL3();

  public Trigger setLocationL4();

  public Trigger autoscoreLeft();

  public Trigger autoscoreRight();

  public Trigger manualScore();

  public Trigger climb();

  public Trigger algaeIntakeOuttake();

  public Trigger algaeDescore();

  public Trigger zeroElevator();

  public Trigger toggleVision();

  public Trigger coralEject();
}
