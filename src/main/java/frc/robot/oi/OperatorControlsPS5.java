package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControlsPS5 implements OperatorControls {
  CommandPS5Controller m_controller;

  public OperatorControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public Trigger coralOuttake() {
    return m_controller.R1();
  }

  @Override
  public Trigger setLocationL1() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setLocationL2() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger setLocationL3() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setLocationL4() {
    return m_controller.povRight();
  }

  @Override
  public Trigger manualScore() {
    return m_controller.R2();
  }

  @Override
  public Trigger climb() {
    return m_controller.cross();
  }

  @Override
  public Trigger algaeDescore() {
    return m_controller.L2();
  }

  @Override
  public Trigger zeroElevator() {
    return m_controller.circle();
  }

  @Override
  public Trigger coralEject() {
    return m_controller.square();
  }

  @Override
  public Trigger toggleOtbRunthrough() {
    return m_controller.L1();
  }

  @Override
  public Trigger zeroClimb() {
    return m_controller.triangle();
  }
}
