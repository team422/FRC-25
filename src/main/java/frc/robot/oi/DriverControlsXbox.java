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
    return -m_controller.getLeftY();
  }

  @Override
  public double getStrafe() {
    return -m_controller.getLeftX();
  }

  @Override
  public double getTurn() {
    return -m_controller.getRightX();
  }

  @Override
  public Trigger resetFieldCentric() {
    return m_controller.back();
  }

  @Override
  public Trigger coralIntake() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger autoscoreLeft() {
    return m_controller.x();
  }

  @Override
  public Trigger autoscoreRight() {
    return m_controller.b();
  }

  @Override
  public Trigger otbMagic() {
    return m_controller.rightTrigger(0.1);
  }

  @Override
  public Trigger toggleVision() {
    return m_controller.start();
  }
}
