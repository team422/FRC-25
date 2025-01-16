package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
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

  @Override
  public Trigger flywheelIdle() {
    return m_controller.a();
  }

  @Override
  public Trigger flywheelSetpoint1() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger flywheelSetpoint2() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger flywheelEject() {
    return m_controller.b();
  }
}
