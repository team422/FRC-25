package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TestingController {
  private CommandXboxController m_controller;

  public TestingController(int port) {
    m_controller = new CommandXboxController(port);
  }

  public Trigger toggleTestingMode() {
    return m_controller.a();
  }

  public Trigger autoAutoscoreLeft() {
    return m_controller.x();
  }

  public Trigger autoAutoscoreRight() {
    return m_controller.b();
  }

  public Trigger autoCoralIntake() {
    return m_controller.rightBumper();
  }

  public Trigger autoLeft() {
    return m_controller.povLeft();
  }

  public Trigger autoRight() {
    return m_controller.povRight();
  }

  public Trigger incrementCoralScored() {
    return m_controller.povUp();
  }

  public Trigger decrementCoralScored() {
    return m_controller.povDown();
  }
}
