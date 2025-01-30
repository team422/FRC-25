package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Ports;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXbox;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOReplay;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOReplay;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private Drive m_drive;
  private Elevator m_elevator;
  private AprilTagVision m_aprilTagVision;

  // Controller
  private DriverControls m_driverControls;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureButtonBindings();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    switch (Constants.kCurrentMode) {
      case REAL:
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));

        m_elevator =
            new Elevator(new ElevatorIOKraken(Ports.kElevatorLead, Ports.kElevatorFollowing));

        break;

      case SIM:
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        m_elevator = new Elevator(new ElevatorIOSim());

        break;

      case REPLAY:
        m_drive =
            new Drive(
                new GyroIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay(),
                new ModuleIOReplay());

        m_elevator = new Elevator(new ElevatorIOReplay());

        break;
    }

    m_aprilTagVision =
        new AprilTagVision(
            new AprilTagVisionIONorthstar("northstar_0", ""),
            new AprilTagVisionIONorthstar("northstar_1", ""),
            new AprilTagVisionIONorthstar("northstar_2", ""),
            new AprilTagVisionIONorthstar("northstar_3", ""));

    RobotState.startInstance(m_drive, m_elevator, m_aprilTagVision);
  }

  /** Configure the commands. */
  private void configureCommands() {
    // Auto commands
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoChooser.addOption("Do Nothing", Commands.none());
    m_autoChooser.addOption(
        "Drive Quasistatic Characterization",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive Dynamic Characterization", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
  }

  /** Configure the controllers. */
  private void configureControllers() {
    m_driverControls = new DriverControlsXbox(0);
    // m_driverControls = new DriverControlsPS5(0);
  }

  /** Configure the button bindings. */
  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_driverControls::getForward,
            m_driverControls::getStrafe,
            m_driverControls::getTurn));
    m_driverControls
        .elevatorStow()
        .onTrue(Commands.runOnce(() -> m_elevator.updateState(ElevatorState.kStow)));
    m_driverControls
        .elevatorScoring()
        .onTrue(Commands.runOnce(() -> m_elevator.updateState(ElevatorState.kScoring)));
    m_driverControls
        .elevatorIntaking()
        .onTrue(Commands.runOnce(() -> m_elevator.updateState(ElevatorState.kIntaking)));
    m_driverControls
        .elevatorKnocking()
        .onTrue(Commands.runOnce(() -> m_elevator.updateState(ElevatorState.kKnocking)));
    m_driverControls
        .incrementScoringLocation()
        .onTrue(
            Commands.runOnce(
                () -> {
                  var curr = m_elevator.getDesiredScoringLocation();
                  FieldConstants.ReefHeight[] heights = FieldConstants.ReefHeight.values();
                  int val = curr.ordinal();
                  val = (val + 1) % heights.length;
                  m_elevator.setDesiredScoringLocation(heights[val]);
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
