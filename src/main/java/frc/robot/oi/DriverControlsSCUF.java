package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsSCUF implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsSCUF(int port) {
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
    return -m_controller.getRawAxis(4);
  }

  @Override
  public Trigger resetFieldCentric() {
    return m_controller.a();
  }

  @Override
  public Trigger coralIntake() {
    return m_controller.leftTrigger(0.1);
  }

  @Override
  public Trigger autoscoreLeft() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger autoscoreRight() {
    return m_controller.rightBumper();
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
