package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToPoint extends Command {
  private Drive m_drive;
  private Pose2d m_targetPose;

  private ProfiledPIDController m_driveController =
      new ProfiledPIDController(
          DriveConstants.kDriveToPointP.get(),
          DriveConstants.kDriveToPointI.get(),
          DriveConstants.kDriveToPointD.get(),
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxLinearSpeed, DriveConstants.kMaxLinearAcceleration));

  private ProfiledPIDController m_headingController =
      new ProfiledPIDController(
          DriveConstants.kDriveToPointHeadingP.get(),
          DriveConstants.kDriveToPointHeadingI.get(),
          DriveConstants.kDriveToPointHeadingD.get(),
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAcceleration));

  private double m_ffMinRadius = 0.35, m_ffMaxRadius = 0.8;

  public DriveToPoint(Drive drive, Pose2d targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;

    m_headingController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController.setTolerance(Units.inchesToMeters(0.5));
    m_headingController.setTolerance(Units.degreesToRadians(1));
  }

  public void setTargetPose(Pose2d targetPose) {
    m_targetPose = targetPose;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = m_drive.getPose();
    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drive.getChassisSpeeds(), currentPose.getRotation());
    m_driveController.reset(
        currentPose.getTranslation().getDistance(m_targetPose.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
                .rotateBy(
                    m_targetPose
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    m_headingController.reset(
        currentPose.getRotation().getRadians(), fieldRelative.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_driveController.setP(DriveConstants.kDriveToPointP.get());
          m_driveController.setI(DriveConstants.kDriveToPointI.get());
          m_driveController.setD(DriveConstants.kDriveToPointD.get());
          m_headingController.setP(DriveConstants.kDriveToPointHeadingP.get());
          m_headingController.setI(DriveConstants.kDriveToPointHeadingI.get());
          m_headingController.setD(DriveConstants.kDriveToPointHeadingD.get());
        },
        DriveConstants.kDriveToPointP,
        DriveConstants.kDriveToPointI,
        DriveConstants.kDriveToPointD,
        DriveConstants.kDriveToPointHeadingP,
        DriveConstants.kDriveToPointHeadingI,
        DriveConstants.kDriveToPointHeadingD);

    Pose2d currentPose = m_drive.getPose();

    double currentDistance =
        currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - m_ffMinRadius) / (m_ffMaxRadius - m_ffMinRadius), 0.0, 1.0);
    double driveVelocityScalar =
        m_driveController.getSetpoint().velocity * ffScaler
            + m_driveController.calculate(currentDistance, 0.0);
    if (currentDistance < m_driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    double headingError = currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians();
    double headingVelocity =
        m_headingController.getSetpoint().velocity * ffScaler
            + m_headingController.calculate(
                currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    if (Math.abs(headingError) < m_headingController.getPositionTolerance()) {
      headingVelocity = 0.0;
    }

    // evil math
    // blame 254 for making this because i dont fully understand it
    // update: i understand it now :)
    Translation2d driveVelocity =
        new Pose2d(
                0.0,
                0.0,
                currentPose.getTranslation().minus(m_targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), headingVelocity, currentPose.getRotation());
    m_drive.setDesiredChassisSpeeds(speeds);

    Logger.recordOutput("DriveToPoint/TargetPose", m_targetPose);
    Logger.recordOutput("DriveToPoint/DriveDistance", currentDistance);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);

    Logger.recordOutput("DriveToPoint/FFScaler", ffScaler);
    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);
  }

  @Override
  public boolean isFinished() {
    return m_driveController.atGoal() && m_headingController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setDesiredChassisSpeeds(new ChassisSpeeds());
  }
}
