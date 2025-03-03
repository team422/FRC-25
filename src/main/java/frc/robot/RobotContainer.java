package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.Controls;
import frc.robot.oi.ControlsPS5;
// import frc.robot.oi.ControlsXbox;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOCIM;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIOCIM;
import frc.robot.subsystems.roller.Roller.RollerProfiles;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Drive m_drive;
  private Roller m_roller;
  private Controls m_controller;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureControllers();
    configureBindings();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    m_drive = new Drive(new DriveIOCIM());
    m_roller = new Roller(new RollerIOCIM());
    RobotState.startInstance(m_drive, m_roller);
  }

  private void configureControllers() {
    m_controller = new ControlsPS5(0);
    // m_controller = new ControlsXbox(0);
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.arcadeDrive(m_drive, m_controller::getMovement, m_controller::getRotation));
        
    m_controller.roller().onTrue(
      Commands.runOnce(() -> {
        m_roller.updateState(RollerProfiles.kOuttake);
      })
    ).onFalse(
      Commands.runOnce(() -> {
        m_roller.updateState(RollerProfiles.kIdle);
      })
    );
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
