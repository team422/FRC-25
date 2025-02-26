package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.littletonUtils.LocalADStarAK;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants.ReefHeight;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AutoFactory {
  public static final PIDConstants kLinearPID = new PIDConstants(3.0, 0.0, 0.0);
  public static final PIDConstants kAngularPID = new PIDConstants(1.0, 0.0, 0.05);

  private final Drive m_drive;

  public AutoFactory(Drive drive) {
    m_drive = drive;

    NamedCommands.registerCommand(
        "Idle",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDefaultAction();
            }));

    NamedCommands.registerCommand(
        "Coral Intake",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().updateRobotAction(RobotAction.kCoralIntaking);
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

    NamedCommands.registerCommand(
        "Set Height L4",
        Commands.runOnce(
            () -> {
              RobotState.getInstance().setDesiredReefHeight(ReefHeight.L4);
            }));

    NamedCommands.registerCommand(
        "Autoscore Left",
        Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexLeft();
                })
            .andThen(new AutoAutoScore()));

    NamedCommands.registerCommand(
        "Autoscore Right",
        Commands.runOnce(
                () -> {
                  RobotState.getInstance().setReefIndexRight();
                })
            .andThen(new AutoAutoScore()));

    NamedCommands.registerCommand("Coral Intake", new AutoCoralIntake());

    AutoBuilder.configure(
        m_drive::getPose,
        (Pose2d pose) -> {},
        m_drive::getChassisSpeeds,
        m_drive::setDesiredChassisSpeeds,
        new PPHolonomicDriveController(kLinearPID, kAngularPID, 0.02),
        new RobotConfig(
            DriveConstants.kRobotMass,
            DriveConstants.kRobotMOI,
            new ModuleConfig(
                DriveConstants.kWheelRadius,
                DriveConstants.kMaxLinearSpeed,
                1.0,
                DCMotor.getKrakenX60Foc(1),
                CurrentLimitConstants.kDriveDefaultSupplyCurrentLimit,
                4),
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
    return autoCommand.andThen(Commands.runOnce(m_drive::stopWithX));
  }
}
