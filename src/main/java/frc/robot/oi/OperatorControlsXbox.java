package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControlsXbox implements OperatorControls {
  CommandXboxController m_controller;

  public OperatorControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public Trigger coralOuttake() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger setLocationL1() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setLocationL2() {
    return m_controller.povRight();
  }

  @Override
  public Trigger setLocationL3() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setLocationL4() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger manualScore() {
    return m_controller.rightTrigger(0.1);
  }

  @Override
  public Trigger climb() {
    return m_controller.a();
  }

  @Override
  public Trigger algaeDescore() {
    return m_controller.leftTrigger(0.1);
  }

  @Override
  public Trigger zeroElevator() {
    return m_controller.b();
  }

  @Override
  public Trigger coralEject() {
    return m_controller.x();
  }

  @Override
  public Trigger toggleOtbRunthrough() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger zeroClimb() {
    return m_controller.y();
  }
}
