package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
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
    return m_controller.y();
  }

  @Override
  public Trigger outtake() {
    return m_controller.rightBumper();
  }
}
