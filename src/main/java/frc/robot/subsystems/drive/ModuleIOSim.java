package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two DC motor sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The DC motor sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotor m_driveMotor = DCMotor.getKrakenX60(1);
  private DCMotor m_turnMotor = DCMotor.getKrakenX60(1);

  private DCMotorSim m_driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              m_driveMotor, DriveConstants.kDriveSimMOI, DriveConstants.kDriveSimGearRatio),
          m_driveMotor);
  private DCMotorSim m_turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              m_turnMotor, DriveConstants.kTurnSimMOI, DriveConstants.kTurnSimGearRatio),
          m_turnMotor);

  private final Rotation2d m_turnAbsoluteInitPosition =
      new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double m_driveAppliedVolts = 0.0;
  private double m_driveFeedforward = 0.0;
  private double m_turnAppliedVolts = 0.0;

  private PIDController m_driveFeedback = new PIDController(0.0, 0.0, 0.0);
  private PIDController m_turnFeedback = new PIDController(0.0, 0.0, 0.0);

  {
    m_turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  // whether to use voltage control or pid control
  private boolean m_driveVelocityControl = false;
  private boolean m_turnPositionControl = false;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (m_driveVelocityControl) {
      m_driveAppliedVolts =
          m_driveFeedback.calculate(m_driveSim.getAngularVelocityRadPerSec()) + m_driveFeedforward;
    }

    if (m_turnPositionControl) {
      m_turnAppliedVolts = m_turnFeedback.calculate(m_turnSim.getAngularPositionRad());
    }

    m_driveSim.setInputVoltage(m_driveAppliedVolts);
    m_turnSim.setInputVoltage(m_turnAppliedVolts);

    m_driveSim.update(LOOP_PERIOD_SECS);
    m_turnSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(m_turnSim.getAngularPositionRad()).plus(m_turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    m_driveFeedback.setSetpoint(velocityRadPerSec);
    m_driveFeedforward = feedforward;
    m_driveVelocityControl = true;
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    m_turnFeedback.setSetpoint(position.getRadians());
    m_turnPositionControl = true;
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void setTurnBrakeMode(boolean enable) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public void setDriveRawOutput(double output) {
    m_driveAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
    m_driveVelocityControl = false;
  }

  @Override
  public void setTurnRawOutput(double output) {
    m_turnAppliedVolts = MathUtil.clamp(output, -12.0, 12.0);
    m_turnPositionControl = false;
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    m_turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void resetTurnMotor(Angle position) {}
}
