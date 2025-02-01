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
  public Trigger level1() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'level1'");
  }

  @Override
  public Trigger level2() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'level2'");
  }

  @Override
  public Trigger level3() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'level3'");
  }

  @Override
  public Trigger level4() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'level4'");
  }

  @Override
  public Trigger runIndexer() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runIndexer'");
  }

  @Override
  public Trigger intake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'intake'");
  }

  @Override
  public Trigger manipulate() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'manipulate'");
  }

  @Override
  public Trigger stowElevator() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stowElevator'");
  }
}
