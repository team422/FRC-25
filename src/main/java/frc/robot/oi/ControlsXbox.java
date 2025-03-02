package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlsXbox implements Controls {
  private CommandXboxController m_controller;

  public ControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public double getMovement() {
    return m_controller.getLeftY();
  }

  @Override
  public double getRotation() {
    return m_controller.getRightX();
  }

  @Override
  public Trigger roller() {
    return m_controller.rightTrigger();
  }
}
