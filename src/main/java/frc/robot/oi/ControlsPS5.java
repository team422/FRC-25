package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class ControlsPS5 implements Controls {
  private CommandPS5Controller m_controller;

  public ControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public double getMovement() {
    return m_controller.getLeftY();
  }

  @Override
  public double getRotation() {
    return m_controller.getRightX();
  }
}
