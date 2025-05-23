package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ProtoConstants;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.oi.TestingController;
import frc.robot.subsystems.aprilTagVision.AprilTagVision;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbState;
import frc.robot.subsystems.climb.ClimbIOKraken;
import frc.robot.subsystems.climb.ClimbIOReplay;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
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
import frc.robot.subsystems.manipulator.Manipulator.ManipulatorState;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIOPhotoelectric;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIOReplay;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIOKraken;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIOReplay;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIOSim;
import frc.robot.subsystems.manipulator.wrist.WristIOKraken;
import frc.robot.subsystems.manipulator.wrist.WristIOReplay;
import frc.robot.subsystems.manipulator.wrist.WristIOSim;
import frc.robot.util.PathPlannerUtil;
import java.util.List;
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
  private TestingController m_testingController;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  private AutoFactory m_autoFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureButtonBindings();
  }

  /** Configure the subsystems. */
  private void configureSubsystems() {
    m_aprilTagVision =
        new AprilTagVision(
            new AprilTagVisionIONorthstar("northstar_0", ""),
            new AprilTagVisionIONorthstar("northstar_1", ""),
            new AprilTagVisionIONorthstar("northstar_2", ""),
            new AprilTagVisionIONorthstar("northstar_3", ""));

    switch (Constants.kCurrentMode) {
      case REAL:
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));

        // m_intake = new Intake(new IntakeRollerIOReplay(), new PivotIOReplay());
        m_intake =
            new Intake(
                new IntakeRollerIOKraken(Ports.kIntakeRoller),
                new PivotIOKraken(Ports.kIntakePivot, Ports.kIntakeAbsoluteEncoder));

        m_indexer =
            new Indexer(new IndexerIOKraken(Ports.kIndexerSideMotor, Ports.kIndexerTopMotor));
        // m_indexer = new Indexer(new IndexerIOReplay());

        m_manipulator =
            new Manipulator(
                new ManipulatorRollerIOKraken(Ports.kManipulatorRoller),
                new WristIOKraken(Ports.kManipulatorWrist, Ports.kManipulatorAbsoluteEncoder),
                new CoralDetectorIOPhotoelectric(
                    Ports.kManipulatorPhotoElectricOne,
                    Ports.kManipulatorPhotoElectricTwo,
                    Ports.kFunnelPhotoElectricOne,
                    Ports.kFunnelPhotoElectricTwo));
        // new ManipulatorRollerIOReplay(), new WristIOReplay(), new CoralDetectorIOReplay());

        m_climb = new Climb(new ClimbIOKraken(Constants.Ports.kClimbMotor));
        // m_climb = new Climb(new ClimbIOReplay());

        m_elevator =
            new Elevator(new ElevatorIOKraken(Ports.kElevatorLead, Ports.kElevatorFollowing));
        // new Elevator(new ElevatorIOReplay());

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
          m_indexer =
              new Indexer(new IndexerIOKraken(Ports.kIndexerSideMotor, Ports.kIndexerTopMotor));
        } else {
          m_indexer = new Indexer(new IndexerIOSim());
        }

        if (ProtoConstants.kRealManipulator) {
          m_manipulator =
              new Manipulator(
                  new ManipulatorRollerIOKraken(Ports.kManipulatorRoller),
                  new WristIOKraken(Ports.kManipulatorWrist, Ports.kManipulatorAbsoluteEncoder),
                  new CoralDetectorIOPhotoelectric(
                      Ports.kManipulatorPhotoElectricOne,
                      Ports.kManipulatorPhotoElectricTwo,
                      Ports.kFunnelPhotoElectricOne,
                      Ports.kFunnelPhotoElectricTwo));
        } else {
          m_manipulator =
              new Manipulator(
                  new ManipulatorRollerIOSim(),
                  new WristIOSim(),
                  new CoralDetectorIOPhotoelectric(
                      Ports.kManipulatorPhotoElectricOne,
                      Ports.kManipulatorPhotoElectricTwo,
                      Ports.kFunnelPhotoElectricOne,
                      Ports.kFunnelPhotoElectricTwo));
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
                new CoralDetectorIOPhotoelectric(
                    Ports.kManipulatorPhotoElectricOne,
                    Ports.kManipulatorPhotoElectricTwo,
                    Ports.kFunnelPhotoElectricOne,
                    Ports.kFunnelPhotoElectricTwo));

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
  }

  /** Configure the commands. */
  private void configureCommands() {
    // Auto commands
    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    m_autoFactory = new AutoFactory(m_drive);

    m_autoChooser.addOption("Do Nothing", Commands.none());
    m_autoChooser.addOption(
        "Drive Feedforward Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    m_autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));

    m_autoChooser.addOption(
        "Drive SysId Quasistatic", m_drive.sysIdQuasistatic(Direction.kForward));
    m_autoChooser.addOption("Drive SysId Dynamic", m_drive.sysIdDynamic(Direction.kForward));

    m_autoChooser.addDefaultOption("3 Coral Left", m_autoFactory.getAutoCommand("3 Coral Left"));

    List<String> paths = PathPlannerUtil.getExistingPaths();
    for (String path : paths) {
      m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
    }

    // we start here so autofactory won't be null
    RobotState.startInstance(
        m_drive,
        m_intake,
        m_indexer,
        m_manipulator,
        m_climb,
        m_elevator,
        m_led,
        m_aprilTagVision,
        m_autoFactory);
  }

  /** Configure the controllers. */
  private void configureControllers() {
    // m_driverControls = new DriverControlsXbox(0);
    m_driverControls = new DriverControlsPS5(0);
    m_testingController = new TestingController(5);
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
                () ->
                    m_drive.setPose(
                        new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d()))));

    // if anyone is reading this, here's a lesson
    // don't use wpilib toggle, it's not good
    m_driverControls
        .coralIntake()
        .toggleOnTrue(
            Commands.runOnce(
                    () -> {
                      RobotState.getInstance().updateRobotAction(RobotAction.kCoralIntaking);
                    })
                .andThen(Commands.idle())
                .onlyWhile(
                    () -> RobotState.getInstance().getCurrentAction() == RobotAction.kCoralIntaking)
                .handleInterrupt(
                    () -> {
                      RobotState.getInstance().setDefaultAction();
                    }));

    m_driverControls
        .coralOuttake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().manageCoralOuttakePressed();
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
                  // if we are in autoscore and we are on a left branch (autoscore left already
                  // pressed), then cancel
                  if ((RobotState.getInstance().getCurrentAction() == RobotAction.kAutoScore
                      && RobotState.getInstance().getDesiredBranchIndex() % 2 == 1)) {
                    RobotState.getInstance().setDefaultAction();
                  } else if (RobotState.getInstance().getCurrentAction()
                          == RobotAction.kBargeAutoScore
                      && RobotState.getInstance().getBargeLeftCage()) {
                    m_manipulator.updateState(ManipulatorState.kStow);
                    RobotState.getInstance().setDefaultAction();
                  } else if (RobotState.getInstance().getManipulatorState()
                      == ManipulatorState.kAlgaeHold) {
                    RobotState.getInstance().setBargeScoreLeft();
                    RobotState.getInstance().updateRobotAction(RobotAction.kBargeAutoScore);
                  } else {
                    RobotState.getInstance().setReefIndexLeft();
                    RobotState.getInstance().manageAutoScoreButton();
                  }
                }));

    m_driverControls
        .autoscoreRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  // if we are in autoscore and we are on a right branch (autoscore right already
                  // pressed), then cancel
                  if ((RobotState.getInstance().getCurrentAction() == RobotAction.kAutoScore
                      && RobotState.getInstance().getDesiredBranchIndex() % 2 == 0)) {
                    RobotState.getInstance().setDefaultAction();
                  } else if (RobotState.getInstance().getCurrentAction()
                          == RobotAction.kBargeAutoScore
                      && !RobotState.getInstance().getBargeLeftCage()) {
                    m_manipulator.updateState(ManipulatorState.kStow);
                    RobotState.getInstance().setDefaultAction();
                  } else if (RobotState.getInstance().getManipulatorState()
                      == ManipulatorState.kAlgaeHold) {
                    RobotState.getInstance().setBargeScoreRight();
                    RobotState.getInstance().updateRobotAction(RobotAction.kBargeAutoScore);
                  } else {
                    RobotState.getInstance().setReefIndexRight();
                    RobotState.getInstance().manageAutoScoreButton();
                  }
                }));

    m_driverControls
        .manualScore()
        .onTrue(
            Commands.runOnce(
                () -> {
                  // this is much simpler, we can just check if we are in manual score and cancel
                  if (RobotState.getInstance().getCurrentAction() == RobotAction.kManualScore) {
                    RobotState.getInstance().setDefaultAction();
                  } else if (RobotState.getInstance().getCurrentAction()
                      == RobotAction.kBargeScore) {
                    m_manipulator.updateState(ManipulatorState.kStow);
                    RobotState.getInstance().setDefaultAction();
                  } else {
                    if (RobotState.getInstance().getManipulatorState()
                        == ManipulatorState.kAlgaeHold) {
                      RobotState.getInstance().updateRobotAction(RobotAction.kBargeScore);
                    } else {
                      RobotState.getInstance().updateRobotAction(RobotAction.kManualScore);
                    }
                  }
                }));

    m_driverControls
        .climb()
        .onTrue(
            Commands.runOnce(
                () -> {
                  // this isn't gonna be in robot state because that's too much work
                  ClimbState newState =
                      m_climb.getCurrentState() == ClimbState.kDeploy
                          ? ClimbState.kStow
                          : ClimbState.kDeploy;
                  m_climb.updateState(newState);
                }));

    m_driverControls
        .otbMagic()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().manageAlgaeIntake();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().manageAlgaeIntakeRelease();
                }));

    m_driverControls
        .algaeDescore()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().algaeDescore();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  // we just assume that we have an algae until something else cancels it
                  RobotState.getInstance().manageAlgaeDescoreRelease();
                }));

    m_driverControls
        .zeroElevator()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      m_elevator.zeroElevator();
                    })
                .ignoringDisable(true));

    m_driverControls
        .toggleVision()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      RobotState.getInstance().toggleUsingVision();
                    })
                .ignoringDisable(true));

    m_driverControls
        .coralEject()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kCoralEject);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDefaultAction();
                }));

    m_driverControls
        .toggleOtbRunthrough()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      RobotState.getInstance().toggleOtbRunthrough();
                    })
                .ignoringDisable(true));

    m_driverControls
        .zeroClimb()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      m_climb.zeroEncoder();
                      m_climb.updateState(ClimbState.kMatch);
                    })
                .ignoringDisable(true));

    m_testingController
        .toggleTestingMode()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().toggleTestingMode();
                }));

    m_testingController
        .autoAutoscoreLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexLeft();
                  RobotState.getInstance().testAutoAutoScore();
                }));
    m_testingController
        .autoAutoscoreRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexRight();
                  RobotState.getInstance().testAutoAutoScore();
                }));

    m_testingController
        .autoCoralIntake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().testAutoCoralIntake();
                }));

    m_testingController
        .autoLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setAutoSideLeft();
                }));

    m_testingController
        .autoRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setAutoSideRight();
                }));

    m_testingController
        .incrementCoralScored()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().incrementCoralScoredAuto();
                }));

    m_testingController
        .decrementCoralScored()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().decrementCoralScoredAuto();
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
