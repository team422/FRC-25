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
  public Trigger elevatorStow() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'elevatorStow'");
  }

  @Override
  public Trigger elevatorScoring() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'elevatorScoring'");
  }

  @Override
  public Trigger elevatorIntaking() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'elevatorIntaking'");
  }

  @Override
  public Trigger elevatorKnocking() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'elevatorKnocking'");
  }

  @Override
  public Trigger incrementScoringLocation() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'incrementScoringLocation'");
  }
}
