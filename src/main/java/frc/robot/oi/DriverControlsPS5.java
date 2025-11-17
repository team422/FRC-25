package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public Trigger setL1() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setL2() {
    return m_controller.povLeft();
  }

  @Override
  public Trigger setL3() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setL4() {
    return m_controller.povRight();
  }

  @Override
  public Trigger manualScore() {
    return m_controller.triangle();
  }

  @Override
  public Trigger outtake() {
    return m_controller.R1();
  }

  @Override
  public Trigger zero() {
    return m_controller.cross();
  }

  @Override
  public Trigger intake() {
    return m_controller.square();
  }
}
