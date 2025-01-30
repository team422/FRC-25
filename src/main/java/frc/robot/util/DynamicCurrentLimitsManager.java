package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.RobotState.RobotAction;

public class DynamicCurrentLimitsManager {
  // singleton instance
  private static DynamicCurrentLimitsManager instance = null;
  private static double CURRENT_BUDGET = 400;
  private static int currentBrownoutCount = 0;
  private PowerDistribution pdp;

  // states for each ratio

  public RobotAction[] driveActions = {RobotAction.kTeleopDefault, RobotAction.kAutoDefault};
  // public RobotAction[] shootingActions = {
  //
  // RobotAction.kRevAndAlign,RobotAction.kAutoRevAndAutoAlign,RobotAction.kAutoSOTM,RobotAction.kNothing,RobotAction.kAutoSOTM, RobotAction.kAutoShootAtPosition
  // };
  // public RobotAction[] intakeActions = {
  //     RobotAction.kIntake,RobotAction.kAutoIntake,RobotAction.kGamePieceLock
  // };

  // private constructor
  private DynamicCurrentLimitsManager() {
    pdp = new PowerDistribution();
  }

  // get instance
  public static DynamicCurrentLimitsManager getInstance() {
    if (instance == null) {
      instance = new DynamicCurrentLimitsManager();
    }
    return instance;
  }

  public void update() {
    // frc get brownout count

    if (pdp.getFaults().Brownout) {
      currentBrownoutCount++;
      CURRENT_BUDGET = Math.max(CURRENT_BUDGET - 10, 150);
    }

    // if robot is in a drive action

    for (RobotAction action : driveActions) {
      if (frc.robot.RobotState.getInstance().getCurrentAction() == action) {
        // TODO: increase minimums
        double moduleBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.7 / 4), 30);
        double shooterBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.15), 25);
        double shooterPivotBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.05), 10);

        // frc.robot.RobotState.getInstance().setCurrentLimits(moduleBudget, shooterBudget,
        // shooterPivotBudget);
      }
    }

    // // if robot is in a shooting action
    // for (RobotAction action : shootingActions) {
    //     if (frc.robot.RobotState.getInstance().curAction == action) {
    //         double moduleBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.5/4),20);
    //         double shooterBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.4),50);
    //         double shooterPivotBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.1),15);

    //         frc.robot.RobotState.getInstance().setCurrentLimits(moduleBudget, shooterBudget,
    // shooterPivotBudget);
    //     }
    // }

    // // if robot is in an intake action
    // for (RobotAction action : intakeActions) {
    //     if (frc.robot.RobotState.getInstance().curAction == action) {
    //         double moduleBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.8/4),35);
    //         double shooterBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.1),25);
    //         double shooterPivotBudget = Math.max(Math.floor(CURRENT_BUDGET * 0.05),15);

    //         frc.robot.RobotState.getInstance().setCurrentLimits(moduleBudget, shooterBudget,
    // shooterPivotBudget);
    //     }
    // }

  }
}
