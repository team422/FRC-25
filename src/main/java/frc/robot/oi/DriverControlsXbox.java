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
  public Trigger level1() {
    return m_controller.button(1);
  }

  @Override
  public Trigger level2() {
    return m_controller.button(2);
  }

  @Override
  public Trigger level3() {
    return m_controller.button(3);
  }

  @Override
  public Trigger level4() {
    return m_controller.button(4);
  }

  @Override
  public Trigger runIndexer() {
    return m_controller.button(6);
  }

  @Override
  public Trigger intake() {
    return m_controller.button(7);
  }

  @Override
  public Trigger manipulate() {
    return m_controller.button(8);
  }

  @Override
  public Trigger stowElevator() {
    return m_controller.button(5);
  }
}
