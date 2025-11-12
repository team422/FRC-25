package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.littletonUtils.LocalADStarAK;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.subsystems.manipulator.Manipulator;
import org.littletonrobotics.junction.Logger;

public class AutoFactory {

  public static final PIDConstants kLinearPID = new PIDConstants(3.0, 0.0, 0.0);
  public static final PIDConstants kAngularPID = new PIDConstants(0.0, 0.0, 0.00);

  private final Drive m_drive;

  public AutoFactory(Drive drive, Manipulator manipulator) {
    m_drive = drive;

    NamedCommands.registerCommand(
        "Idle",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDefaultAction();
            }));

    NamedCommands.registerCommand(
        "Set Height L1",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDesiredReefHeight(ReefHeight.L1);
            }));

    NamedCommands.registerCommand(
        "Set Height L2",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDesiredReefHeight(ReefHeight.L2);
            }));

    NamedCommands.registerCommand(
        "Set Height L3",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDesiredReefHeight(ReefHeight.L3);
            }));

    // NamedCommands.registerCommand(
    //     "Left side",
    //     Commands.runOnce(
    //         () -> {
    //           RobotState.getInstance().setAutoSideLeft();
    //         }));

    // NamedCommands.registerCommand(
    //     "Right side",
    //     Commands.runOnce(
    //         () -> {
    //           RobotState.getInstance().setAutoSideRight();
    //         }));

    new EventTrigger("Set Height L4")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setDesiredReefHeight(ReefHeight.L4);
                }));

    new EventTrigger("Set Angle 45")
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drive.setDesiredHeading(Rotation2d.fromDegrees(45));
                }));
    new EventTrigger("Set Angle -60")
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drive.setDesiredHeading(Rotation2d.fromDegrees(-60));
                }));

    new EventTrigger("Autoscore Left")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexLeft();
                  RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
                }));
    new EventTrigger("Autoscore Right")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexRight();
                  RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
                }));

    NamedCommands.registerCommand(
        "Autoscore Left",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setReefIndexLeft();
              RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
            }));
    NamedCommands.registerCommand(
        "Autoscore Right",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setReefIndexRight();
              RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
            }));
    NamedCommands.registerCommand(
        "Set Height L4",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDesiredReefHeight(ReefHeight.L4);
            }));
    NamedCommands.registerCommand(
        "Drive To Loader",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().driveToProcessorPeriodic();
            }));

    // NamedCommands.registerCommand(
    //     "Barge Auto",
    //     Commands.runOnce(
    //         () -> {
    //           RobotState.getInstance().setBargeAuto(true);
    //         }));

    // NamedCommands.registerCommand(
    //     "Coral Auto",
    //     Commands.runOnce(
    //         () -> {
    //           RobotState.getInstance().setBargeAuto(false);
    //         }));

    // NamedCommands.registerCommand(
    //     "Citrus Auto",
    //     Commands.runOnce(
    //         () -> {
    //           RobotState.getInstance().setCitrusAuto(true);
    //         }));
    // NamedCommands.registerCommand(
    //     "Non-citrus Auto",
    //     Commands.runOnce(
    //         () -> {
    //           RobotState.getInstance().setCitrusAuto(false);
    //         }));

    // NamedCommands.registerCommand(
    //     "autoscoreFinish",
    //     Commands.waitUntil(
    //         () -> {
    //           RobotAction s = RobotState.getInstance().getCurrentAction();
    //           return s != RobotAction.kCoralOuttaking && s != RobotAction.kAutoAutoScore;
    //         }));

    NamedCommands.registerCommand(
        "Intake",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().updateRobotAction(RobotAction.kCoralIntaking);
              m_drive.setDesiredHeading(Rotation2d.fromDegrees(130));
            }));

    NamedCommands.registerCommand(
        "UsePP",
        Commands.runOnce(
            () -> {
              m_drive.updateProfile(DriveProfiles.kPathplanner);
            }));

    NamedCommands.registerCommand(
        "WaitTillScored",
        Commands.waitUntil(
            () -> {
              return !RobotState.getInstance().getCurrentAction().equals(RobotAction.kAutoScore)
                  && !RobotState.getInstance()
                      .getCurrentAction()
                      .equals(RobotAction.kCoralOuttaking);
            }));

    NamedCommands.registerCommand(
        "WaitTillInFunnel",
        Commands.waitUntil(
            () -> {
              return manipulator.gamePieceInFunnel();
            }));

    new EventTrigger("Coral Intake Left")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kCoralIntaking);
                  m_drive.setDesiredHeading(Rotation2d.fromDegrees(130));
                }));

    new EventTrigger("Coral Intake Right")
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotState.getInstance().updateRobotAction(RobotAction.kCoralIntaking);
                  m_drive.setDesiredHeading(Rotation2d.fromDegrees(-130));
                }));

    AutoBuilder.configure(
        m_drive::getPose,
        (pose) -> {},
        m_drive::getChassisSpeeds,
        m_drive::setDesiredAutoChassisSpeeds,
        new PPHolonomicDriveController(kLinearPID, kAngularPID, 0.02),
        new RobotConfig(
            DriveConstants.kRobotMass,
            DriveConstants.kRobotMOI,
            new ModuleConfig(
                DriveConstants.kWheelRadius,
                // DriveConstants.kMaxLinearSpeed,
                99,
                1.7,
                DCMotor.getKrakenX60Foc(1),
                900,
                1),
            Meters.of(DriveConstants.kTrackWidthX)),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_drive // The subsystem that will be used to follow the path
        );

    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public Command getAutoCommand(String name) {
    Command autoCommand = AutoBuilder.buildAuto(name);
    // return autoCommand.andThen(Commands.runOnce(m_drive::stopWithX));
    return autoCommand;
  }

  public Pose2d getStartingPose(String name) {
    return new PathPlannerAuto(name).getStartingPose();
  }
}
