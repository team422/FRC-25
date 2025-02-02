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
  public Trigger robotstateDefault() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'robotstateDefault'");
  }

  @Override
  public Trigger robotstateCoralIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'robotstateCoralIntake'");
  }

  @Override
  public Trigger robotstateAlgaeIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'robotstateAlgaeIntake'");
  }

  @Override
  public Trigger robotstateAlgaeOuttake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'robotstateAlgaeOuttake'");
  }
}
