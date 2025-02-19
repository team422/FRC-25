package frc.robot;

import frc.robot.oi.Controls;
import frc.robot.oi.ControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOCIM;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drive m_drive;
  private Controls m_controls;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureControllers();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    m_drive = new Drive(new DriveIOCIM());

    RobotState.startInstance(m_drive);
  }

  private void configureControllers() {
    m_controls = new ControlsPS5(0);
    // m_controls = new ControlsXbox(0);
  }

  private void configureBindings() {
    /*m_drive.setDefaultCommand(
      DriveCommands.arcadeDrive(
        m_drive,
        m_controls::getMovement,
        m_controls::getRotation));
    */
  }
}
