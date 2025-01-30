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
  public Trigger elevatorStow() {
    return m_controller.button(1);
  }

  @Override
  public Trigger elevatorScoring() {
    return m_controller.button(2);
  }

  @Override
  public Trigger elevatorIntaking() {
    return m_controller.button(3);
  }

  @Override
  public Trigger elevatorKnocking() {
    return m_controller.button(4);
  }

  @Override
  public Trigger incrementScoringLocation() {
    return m_controller.button(5);
  }
}
