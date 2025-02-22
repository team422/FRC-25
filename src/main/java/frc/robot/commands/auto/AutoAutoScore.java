package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;

public class AutoAutoScore extends Command {
  @Override
  public void initialize() {
    RobotState.getInstance().updateRobotAction(RobotAction.kAutoScore);
  }

  @Override
  public boolean isFinished() {
    // we will transition from kAutoAutoScore to kAutoCoralOuttaking once everything is within
    // tolerance, then from kAutoCoralOuttaking to kAutoDefault once the outtaking is done
    return RobotState.getInstance().getCurrentAction() != RobotAction.kAutoAutoScore
        && RobotState.getInstance().getCurrentAction() != RobotAction.kAutoCoralOuttaking;
  }
}
