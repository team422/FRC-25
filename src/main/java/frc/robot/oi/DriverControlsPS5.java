package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public double getForward() {
    return m_controller.getLeftY();
  }

  @Override
  public double getStrafe() {
    return m_controller.getLeftX();
  }

  @Override
  public double getTurn() {
    return m_controller.getRightX();
  }
}
