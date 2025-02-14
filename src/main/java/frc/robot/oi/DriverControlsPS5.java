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
  public Trigger coralIntake() {
    return m_controller.L1();
  }

  @Override
  public Trigger coralOuttake() {
    return m_controller.R1();
  }

  @Override
  public Trigger setLocationL1() {
    return m_controller.povUp();
  }

  @Override
  public Trigger setLocationL2() {
    return m_controller.povRight();
  }

  @Override
  public Trigger setLocationL3() {
    return m_controller.povDown();
  }

  @Override
  public Trigger setLocationL4() {
    return m_controller.povLeft();
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
  public Trigger manualScore() {
    return m_controller.triangle();
  }

  @Override
  public Trigger climb() {
    return m_controller.cross();
  }

  @Override
  public Trigger algaeIntakeOuttake() {
    return m_controller.L2();
  }

  @Override
  public Trigger algaeDescore() {
    return m_controller.R2();
  }
}
