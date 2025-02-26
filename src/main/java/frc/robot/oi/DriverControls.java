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

  public default Trigger cancelDriveToPoint() {
    // if we have any input on the joysticks, we want to cancel the drive to point command
    return new Trigger(() -> getForward() > 0.1 || getStrafe() > 0.1 || getTurn() > 0.1);
  }
}
