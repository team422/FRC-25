package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.detector.CoralDetectorIOReal;
import frc.robot.subsystems.manipulator.rollers.RollerIOKraken;
import frc.robot.subsystems.manipulator.rollers.RollerIOSim;
import frc.robot.subsystems.manipulator.wrist.WristIOKraken;
import frc.robot.subsystems.manipulator.wrist.WristIOSim;
import org.littletonrobotics.junction.Logger;
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
  private Manipulator m_manipulator;
  private Intake m_intake;

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
    if (RobotBase.isReal()) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));
      new Elevator(new ElevatorIOKraken(Ports.kElevatorLead, Ports.kElevatorFollowing));
      m_manipulator =
          new Manipulator(
              new RollerIOKraken(Ports.kManipulatorRoller),
              new WristIOKraken(Ports.kManipulatorWrist),
              new CoralDetectorIOReal(
                  Ports.kManipulatorPhotoElectricOne,
                  Ports.kManipulatorPhotoElectricTwo,
                  Ports.kFunnelPhotoElectricOne,
                  Ports.kFunnelPhotoElectricTwo));
      m_intake = new Intake(new IntakeIOKraken(Ports.kIndexerTopMotor, Ports.kIndexerSideMotor));
    } else {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      m_elevator = new Elevator(new ElevatorIOSim());
      m_manipulator =
          new Manipulator(
              new RollerIOSim(),
              new WristIOSim(),
              new CoralDetectorIOReal(
                  Ports.kManipulatorPhotoElectricOne,
                  Ports.kManipulatorPhotoElectricTwo,
                  Ports.kFunnelPhotoElectricOne,
                  Ports.kFunnelPhotoElectricTwo));
      m_intake = new Intake(new IntakeIOSim());
    }
  }

  /** Configure the commands. */
  private void configureCommands() {
    // Auto commands

    // we start here so autofactory won't be null
    RobotState.startInstance(m_drive, m_elevator, m_manipulator, m_intake);
  }

  /** Configure the controllers. */
  private void configureControllers() {
    // m_driverControls = new DriverControlsXbox(0);
    m_driverControls = new DriverControlsPS5(0);
  }

  /** Configure the button bindings. */
  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_driverControls::getForward,
            m_driverControls::getStrafe,
            m_driverControls::getTurn,
            false));
    m_driverControls
        .resetFieldCentric()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drive.setPose(new Pose2d());
                }));
    m_driverControls
        .setL1()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setHeight(ElevatorConstants.kL1.getAsDouble());
                }));
    m_driverControls
        .setL2()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setHeight(ElevatorConstants.kL2.getAsDouble());
                }));
    m_driverControls
        .setL3()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setHeight(ElevatorConstants.kL3.getAsDouble());
                }));
    m_driverControls
        .setL4()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setHeight(ElevatorConstants.kL4.getAsDouble());
                }));
    m_driverControls
        .manualScore()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getCurrAction() != RobotAction.kScoring
                      && RobotState.getInstance().ready()) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kScoring);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kTeleopDefault);
                  }
                }));
    m_driverControls
        .outtake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("AtSetpoints", RobotState.getInstance().atSetpoints());
                  if (RobotState.getInstance().atSetpoints()
                      && RobotState.getInstance().getCurrAction() == RobotAction.kScoring) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kOuttaking);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kTeleopDefault);
                  }
                }));
    m_driverControls
        .zero()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_elevator.zeroElevator();
                  RobotState.getInstance().setHeight(ElevatorConstants.kStowHeight.get());
                }));
    m_driverControls
        .intake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (RobotState.getInstance().getCurrAction() != RobotAction.kIntaking) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kIntaking);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kTeleopDefault);
                  }
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

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
