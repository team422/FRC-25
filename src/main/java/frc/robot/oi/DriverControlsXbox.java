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
  public Trigger coralIntake() {
    return m_controller.leftBumper();
  }

  @Override
  public Trigger coralOuttake() {
    return m_controller.rightBumper();
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
    return m_controller.x();
  }

  @Override
  public Trigger autoscoreRight() {
    return m_controller.b();
  }

  @Override
  public Trigger manualScore() {
    return m_controller.y();
  }

  @Override
  public Trigger climb() {
    return m_controller.a();
  }

  @Override
  public Trigger algaeIntakeOuttake() {
    return m_controller.leftTrigger(0.1);
  }

  @Override
  public Trigger algaeDescore() {
    return m_controller.rightTrigger(0.1);
  }
}
