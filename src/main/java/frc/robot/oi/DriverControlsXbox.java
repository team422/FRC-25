package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }
}
