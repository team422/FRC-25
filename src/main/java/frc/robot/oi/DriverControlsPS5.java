package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
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
    return m_controller.touchpad();
  }

  @Override
  public Trigger coralIntake() {
    return m_controller.L1();
  }

  @Override
  public Trigger autoscoreLeft() {
    return m_controller.square();
  }

  @Override
  public Trigger autoscoreRight() {
    return m_controller.circle();
  }

  @Override
  public Trigger otbMagic() {
    return m_controller.R2();
  }

  @Override
  public Trigger toggleVision() {
    return m_controller.PS();
  }
}
