package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.littletonUtils.EqualsUtil;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

// TODO: refactor because this code is gross
// A new object should be able to be made every time auto intake or auto net is called
public class MeshedDrivingController {
  private Pose2d desiredPose = null;
  private double debounceAmount;
  private double currentUserControlHead;
  private boolean axisLocked;
  private double axisAngle;
  private double bConstant; // b in mx + b
  private double minX;
  private double maxX;
  private double userControlPrio;

  private double robotMaxSpeed = Constants.DriveConstants.kMaxMeshedSpeed.get();
  private double robotMaxRotationalVelocity = Constants.DriveConstants.kTeleopRotationSpeed.get();

  private PIDController linearXController;
  private PIDController linearYController;
  private PIDController thetaController;

  private ChassisSpeeds speedsOut;
  private ChassisSpeeds speedsIn;

  public MeshedDrivingController(
      Pose2d desiredPose, Boolean axisLocked, double debounce, double controlPrio) {
    this.desiredPose = desiredPose;
    this.debounceAmount = debounce;
    this.axisLocked = axisLocked;
    this.userControlPrio = controlPrio;
  }

  public void setDesiredPose(Pose2d pose) {
    desiredPose = pose;
  }

  public void setPIDControllers(double pXY, double dXY, double pTheta, double dTheta) {
    linearXController = new PIDController(pXY, 0, dXY);
    linearYController = new PIDController(pXY, 0, dXY);
    thetaController = new PIDController(pTheta, 0, dTheta);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // with axis locked controls,
  private void calculateNewPose(Pose2d curPose) {

    // project the closest position on the line
    double b = curPose.getY();
    double a = curPose.getX();
    double m = axisAngle;
    double d = bConstant;

    double c = b + (a / m);
    double newX = (c - d) / (m + (1 / m));

    // Logger.recordOutput("LineMerge", new Pose2d(newX, 0, new Rotation2d()));

    newX = MathUtil.clamp(newX, minX, maxX);
    double newY = m * newX + d;

    desiredPose = new Pose2d(newX, newY, desiredPose.getRotation());
  }

  public void setAxisLocked(double angle, double minX, double maxX, double angleConstant) {
    axisAngle = angle;
    this.minX = minX;
    this.maxX = maxX;
    this.bConstant = angleConstant;
  }

  public ChassisSpeeds calculateSpeeds(ChassisSpeeds userSpeeds, Pose2d curPose) {

    speedsIn = userSpeeds;
    // check if the user is attempting to go against a locked axis
    // if so we must recalculate position
    if (axisLocked) {
      if (!EqualsUtil.epsilonEquals(userSpeeds.vxMetersPerSecond, 0)
          || EqualsUtil.epsilonEquals(userSpeeds.vyMetersPerSecond, 0)) {
        calculateNewPose(curPose);
      }
    }

    // mesh user controls

    if (Constants.DriveConstants.kMaxLinearSpeed.hasChanged(hashCode())) {
      robotMaxSpeed = Constants.DriveConstants.kMaxMeshedSpeed.get();
    }

    if (Constants.DriveConstants.kTeleopRotationSpeed.hasChanged(hashCode())) {
      robotMaxRotationalVelocity = Constants.DriveConstants.kTeleopRotationSpeed.get();
    }

    // lets find the value of user control
    double controlHead =
        MathUtil.clamp(
            Math.pow(
                Math.hypot(userSpeeds.vxMetersPerSecond, userSpeeds.vyMetersPerSecond)
                    / robotMaxSpeed,
                userControlPrio),
            0.0,
            1.0);
    if (controlHead > currentUserControlHead) {
      currentUserControlHead = controlHead;
    } else {
      currentUserControlHead -= debounceAmount;
      currentUserControlHead = MathUtil.clamp(currentUserControlHead, 0., 1.);
    }

    double controlHeadRot =
        MathUtil.clamp(
            Math.abs(userSpeeds.omegaRadiansPerSecond) / robotMaxRotationalVelocity, 0.0, 1.0);

    double thetaPID =
        thetaController.calculate(
            curPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());
    double linearXPID = linearXController.calculate(curPose.getX(), desiredPose.getX());
    double linearYPID = linearYController.calculate(curPose.getY(), desiredPose.getY());

    double Xcombined =
        currentUserControlHead * userSpeeds.vxMetersPerSecond
            + (1 - currentUserControlHead) * linearXPID;
    double Ycombined =
        currentUserControlHead * userSpeeds.vyMetersPerSecond
            + (1 - currentUserControlHead) * linearYPID;
    double thetaCombined =
        controlHeadRot * userSpeeds.omegaRadiansPerSecond + (1 - controlHeadRot) * thetaPID;

    // double velocityPercentage = Math.hypot(Xcombined, Ycombined) / robotMaxSpeed;
    // if (velocityPercentage > 1.0) {
    //   Xcombined = Xcombined / velocityPercentage;
    //   Ycombined = Ycombined / velocityPercentage;
    // }

    Xcombined = MathUtil.applyDeadband(Xcombined, 0.1);
    Ycombined = MathUtil.applyDeadband(Ycombined, 0.1);
    thetaCombined = MathUtil.applyDeadband(thetaCombined, 0.1);

    speedsOut = new ChassisSpeeds(Xcombined, Ycombined, thetaCombined);

    logValues();
    return speedsOut;
  }

  public void logValues() {
    Logger.recordOutput("MeshedControls/minX", minX);
    Logger.recordOutput("MeshedControls/maxX", maxX);
    Logger.recordOutput("MeshedControls/desiredPosition", desiredPose);
    Logger.recordOutput("MeshedControls/currentUserControlHead", currentUserControlHead);
    Logger.recordOutput("MeshedControls/curUserPrio", userControlPrio);
    Logger.recordOutput("MeshedControls/debounce", debounceAmount);
    Logger.recordOutput("MeshedControls/speedsIn", speedsIn);
    Logger.recordOutput("MeshedControls/speedsOut", speedsOut);
  }
}
