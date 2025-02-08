package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ProtoConstants;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXbox;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOKraken;
import frc.robot.subsystems.climb.ClimbIOReplay;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOReplay;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOKraken;
import frc.robot.subsystems.elevator.ElevatorIOReplay;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerIOReplay;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.PivotIOKraken;
import frc.robot.subsystems.intake.pivot.PivotIOReplay;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerIOKraken;
import frc.robot.subsystems.intake.roller.IntakeRollerIOReplay;
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIOPhotoelectric;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIOReplay;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIOKraken;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIOReplay;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIOSim;
import frc.robot.subsystems.manipulator.wrist.WristIOKraken;
import frc.robot.subsystems.manipulator.wrist.WristIOReplay;
import frc.robot.subsystems.manipulator.wrist.WristIOSim;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.stream.Collectors;
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
  private Intake m_intake;
  private Indexer m_indexer;
  private Manipulator m_manipulator;
  private Climb m_climb;
  private Elevator m_elevator;
  private Led m_led;
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
    try {
      // var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
      String resourcePath = AprilTagFields.k2025Reefscape.m_resourceFile;
      InputStream stream = AprilTagFieldLayout.class.getResourceAsStream(resourcePath);
      // String s = AprilTagFields.k2025Reefscape.m_resourceFile;
      // CONVERT STREAM TO STRING
      String s =
          new BufferedReader(new InputStreamReader(stream))
              .lines()
              .collect(Collectors.joining("\n"));
      System.out.println(s);

    } catch (Exception e) {
      e.printStackTrace();
    }

    // // convert to string array lines
    // try {
    //   System.out.println(Files.readAllLines(Path.of(s)));
    // } catch (Exception e) {
    //   e.printStackTrace();
    // }

    switch (Constants.kCurrentMode) {
      case REAL:
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));

        m_intake =
            new Intake(
                new IntakeRollerIOKraken(Ports.kIntakeRoller),
                new PivotIOKraken(Ports.kIntakePivot, Ports.kIntakeAbsoluteEncoder));

        m_indexer = new Indexer(new IndexerIOKraken(Ports.kIndexerMotor));

        m_manipulator =
            new Manipulator(
                new ManipulatorRollerIOKraken(Ports.kManipulatorRoller),
                new WristIOKraken(Ports.kManipulatorWrist, Ports.kManipulatorAbsoluteEncoder),
                new CoralDetectorIOPhotoelectric(Ports.kPhotoElectricOne, Ports.kPhotoElectricTwo));

        m_climb = new Climb(new ClimbIOKraken(Constants.Ports.kClimbMotor));

        m_elevator =
            new Elevator(new ElevatorIOKraken(Ports.kElevatorLead, Ports.kElevatorFollowing));

        break;

      case PROTO:
        if (ProtoConstants.kRealDrive) {
          m_drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3));
        } else {
          m_drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
        }

        if (ProtoConstants.kRealIntake) {
          m_intake =
              new Intake(
                  new IntakeRollerIOKraken(Ports.kIntakeRoller),
                  new PivotIOKraken(Ports.kIntakePivot, Ports.kIntakeAbsoluteEncoder));
        } else {
          m_intake = new Intake(new IntakeRollerIOSim(), new PivotIOSim());
        }

        if (ProtoConstants.kRealIndexer) {
          m_indexer = new Indexer(new IndexerIOKraken(Ports.kIndexerMotor));
        } else {
          m_indexer = new Indexer(new IndexerIOSim());
        }

        if (ProtoConstants.kRealManipulator) {
          m_manipulator =
              new Manipulator(
                  new ManipulatorRollerIOKraken(Ports.kManipulatorRoller),
                  new WristIOKraken(Ports.kManipulatorWrist, Ports.kManipulatorAbsoluteEncoder),
                  new CoralDetectorIOPhotoelectric(
                      Ports.kPhotoElectricOne, Ports.kPhotoElectricTwo));
        } else {
          m_manipulator =
              new Manipulator(
                  new ManipulatorRollerIOSim(),
                  new WristIOSim(),
                  new CoralDetectorIOPhotoelectric(
                      Ports.kPhotoElectricOne, Ports.kPhotoElectricTwo));
        }

        if (ProtoConstants.kRealClimb) {
          m_climb = new Climb(new ClimbIOKraken(Constants.Ports.kClimbMotor));
        } else {
          m_climb = new Climb(new ClimbIOSim());
        }

        if (ProtoConstants.kRealElevator) {
          m_elevator =
              new Elevator(new ElevatorIOKraken(Ports.kElevatorLead, Ports.kElevatorFollowing));
        } else {
          m_elevator = new Elevator(new ElevatorIOSim());
        }

        break;

      case SIM:
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        m_intake = new Intake(new IntakeRollerIOSim(), new PivotIOSim());

        m_indexer = new Indexer(new IndexerIOSim());

        m_manipulator =
            new Manipulator(
                new ManipulatorRollerIOSim(),
                new WristIOSim(),
                new CoralDetectorIOPhotoelectric(Ports.kPhotoElectricOne, Ports.kPhotoElectricTwo));

        m_climb = new Climb(new ClimbIOSim());

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

        m_intake = new Intake(new IntakeRollerIOReplay(), new PivotIOReplay());

        m_indexer = new Indexer(new IndexerIOReplay());

        m_manipulator =
            new Manipulator(
                new ManipulatorRollerIOReplay(), new WristIOReplay(), new CoralDetectorIOReplay());

        m_climb = new Climb(new ClimbIOReplay());

        m_elevator = new Elevator(new ElevatorIOReplay());

        break;
    }

    m_led = new Led(Ports.kLed, LedConstants.kStripLength);

    m_aprilTagVision =
        new AprilTagVision(
            new AprilTagVisionIONorthstar("northstar_0", ""),
            new AprilTagVisionIONorthstar("northstar_1", ""),
            new AprilTagVisionIONorthstar("northstar_2", ""),
            new AprilTagVisionIONorthstar("northstar_3", ""));

    RobotState.startInstance(
        m_drive, m_intake, m_indexer, m_manipulator, m_climb, m_elevator, m_led, m_aprilTagVision);
  }

  /** Configure the commands. */
  private void configureCommands() {
    // Auto commands
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoChooser.addOption("Do Nothing", Commands.none());
    m_autoChooser.addOption(
        "Drive Feedforward Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    m_autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
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
            m_driverControls::getTurn,
            false));

    m_driverControls
        .coralIntake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kCoralIntaking);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDefaultAction();
                }));

    m_driverControls
        .coralOuttake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kCoralOuttaking);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().manageCoralOuttakeRelease();
                }));

    m_driverControls
        .setLocationL1()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDesiredReefHeight(ReefHeight.L1);
                }));

    m_driverControls
        .setLocationL2()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDesiredReefHeight(ReefHeight.L2);
                }));

    m_driverControls
        .setLocationL3()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDesiredReefHeight(ReefHeight.L3);
                }));

    m_driverControls
        .setLocationL4()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDesiredReefHeight(ReefHeight.L4);
                }));

    m_driverControls
        .autoscoreLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexLeft();
                  RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
                }));

    m_driverControls
        .autoscoreRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexRight();
                  RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
                }));

    m_driverControls
        .manualScore()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kManualScore);
                }));

    m_driverControls
        .climb()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kClimbing);
                }));

    m_driverControls
        .algaeIntakeOuttake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kAlgaeIntakingOuttaking);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDefaultAction();
                }));

    m_driverControls
        .cancelDriveToPoint()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (m_drive.getCurrentProfile() == DriveProfiles.kDriveToPoint) {
                    m_drive.updateProfile(DriveProfiles.kDefault);
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
}
