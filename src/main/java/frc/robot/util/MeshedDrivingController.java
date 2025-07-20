package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

/**
 * This class is used to control the robot's movement in a meshed driving style. It takes user input
 * and desired pose and calculates the appropriate speeds for the robot to follow the desired path.
 */
public class MeshedDrivingController {
  private Pose2d m_desiredPose = null;
  private double m_debounceAmount;
  private double m_currentUserControlHead;
  private boolean m_axisLocked;
  private double m_slope;
  private double m_intercept; // b in mx + b
  private double m_minX;
  private double m_maxX;
  private double m_userControlPriority;

  private PIDController m_linearXController;
  private PIDController m_linearYController;
  private PIDController m_thetaController;

  private ChassisSpeeds m_speedsOut;
  private ChassisSpeeds m_speedsIn;

  public MeshedDrivingController(
      Pose2d desiredPose, boolean axisLocked, double debounce, double controlPrio) {
    m_desiredPose = desiredPose;
    m_debounceAmount = debounce;
    m_axisLocked = axisLocked;
    m_userControlPriority = controlPrio;
  }

  public void setDesiredPose(Pose2d pose) {
    m_desiredPose = pose;
    // reset pid
    setPIDControllers(
        m_linearXController.getP(),
        m_linearXController.getD(),
        m_thetaController.getP(),
        m_thetaController.getD());
  }

  public void setPIDControllers(double pXY, double dXY, double pTheta, double dTheta) {
    m_linearXController = new PIDController(pXY, 0, dXY);
    m_linearYController = new PIDController(pXY, 0, dXY);
    m_thetaController = new PIDController(pTheta, 0, dTheta);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // with axis locked controls,
  private void calculateNewPose(Pose2d curPose) {

    // project the closest position on the line
    double b = curPose.getY();
    double a = curPose.getX();
    double m = m_slope;
    double d = m_intercept;

    double c = b + (a / m);
    double newX = (c - d) / (m + (1 / m));

    newX = MathUtil.clamp(newX, m_minX, m_maxX);
    double newY = m * newX + d;

    m_desiredPose = new Pose2d(newX, newY, m_desiredPose.getRotation());
  }

  public void setAxisLocked(double slope, double intercept, double minX, double maxX) {
    m_axisLocked = true;
    m_slope = slope;
    m_intercept = intercept;
    m_minX = minX;
    m_maxX = maxX;
  }

  public void setAxisUnlocked() {
    m_axisLocked = false;
  }

  public ChassisSpeeds calculateSpeeds(ChassisSpeeds userSpeeds, Pose2d curPose) {
    m_speedsIn = userSpeeds;
    // check if the user is attempting to go against a locked axis
    // if so we must recalculate position
    if (m_axisLocked) {
      calculateNewPose(curPose);
    }

    // lets find the value of user control
    double controlHead =
        MathUtil.clamp(
            Math.pow(
                Math.hypot(userSpeeds.vxMetersPerSecond, userSpeeds.vyMetersPerSecond)
                    / DriveConstants.kMaxMeshedSpeed,
                m_userControlPriority),
            0.0,
            1.0);
    if (controlHead > m_currentUserControlHead) {
      m_currentUserControlHead = controlHead;
    } else {
      m_currentUserControlHead -= m_debounceAmount;
      m_currentUserControlHead = MathUtil.clamp(m_currentUserControlHead, 0.0, 1.0);
    }

    double controlHeadRot =
        MathUtil.clamp(
            Math.abs(userSpeeds.omegaRadiansPerSecond) / DriveConstants.kTeleopRotationSpeed.get(),
            0.0,
            1.0);

    double thetaPID =
        m_thetaController.calculate(
            curPose.getRotation().getRadians(), m_desiredPose.getRotation().getRadians());
    double linearXPID = m_linearXController.calculate(curPose.getX(), m_desiredPose.getX());
    double linearYPID = m_linearYController.calculate(curPose.getY(), m_desiredPose.getY());

    double xCombined =
        m_currentUserControlHead * userSpeeds.vxMetersPerSecond
            + (1 - m_currentUserControlHead) * linearXPID;
    double yCombined =
        m_currentUserControlHead * userSpeeds.vyMetersPerSecond
            + (1 - m_currentUserControlHead) * linearYPID;
    double thetaCombined =
        controlHeadRot * userSpeeds.omegaRadiansPerSecond + (1 - controlHeadRot) * thetaPID;

    xCombined = MathUtil.applyDeadband(xCombined, 0.1);
    yCombined = MathUtil.applyDeadband(yCombined, 0.1);
    thetaCombined = MathUtil.applyDeadband(thetaCombined, 0.1);

    m_speedsOut = new ChassisSpeeds(xCombined, yCombined, thetaCombined);

    logValues();

    return m_speedsOut;
  }

  public void logValues() {
    Logger.recordOutput("MeshedControls/minX", m_minX);
    Logger.recordOutput("MeshedControls/maxX", m_maxX);
    Logger.recordOutput("MeshedControls/desiredPosition", m_desiredPose);
    Logger.recordOutput("MeshedControls/currentUserControlHead", m_currentUserControlHead);
    Logger.recordOutput("MeshedControls/curUserPrio", m_userControlPriority);
    Logger.recordOutput("MeshedControls/debounce", m_debounceAmount);
    Logger.recordOutput("MeshedControls/speedsIn", m_speedsIn);
    Logger.recordOutput("MeshedControls/speedsOut", m_speedsOut);
  }
}
