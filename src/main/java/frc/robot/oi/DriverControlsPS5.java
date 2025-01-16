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
    return m_controller.cross();
  }

  @Override
  public Trigger flywheelSetpoint1() {
    return m_controller.R1();
  }

  @Override
  public Trigger flywheelSetpoint2() {
    return m_controller.L1();
  }

  @Override
  public Trigger flywheelEject() {
    return m_controller.circle();
  }
}
