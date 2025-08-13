package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
  public Trigger coralOuttake();

  public Trigger setLocationL1();

  public Trigger setLocationL2();

  public Trigger setLocationL3();

  public Trigger setLocationL4();

  public Trigger manualScore();

  public Trigger climb();

  public Trigger algaeDescore();

  public Trigger zeroElevator();

  public Trigger coralEject();

  public Trigger toggleOtbRunthrough();

  public Trigger zeroClimb();
}
