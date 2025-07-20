package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAccelerationRadPerSecSq = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public boolean driveMotorIsConnected = false;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAccelerationRadPerSecSq = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public boolean turnMotorIsConnected = false;
    public boolean turnEncoderIsConnected = false;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ModuleIOInputs inputs);

  /** Set the desired velocity for the drive motor with a feedforward model. */
  public void setDriveVelocity(double velocityRadPerSec, double feedforward);

  /** Set the desired position for the turn motor. */
  public void setTurnPosition(Rotation2d position);

  /** Enable or disable brake mode on the drive motor. */
  public void setDriveBrakeMode(boolean enable);

  /** Enable or disable brake mode on the turn motor. */
  public void setTurnBrakeMode(boolean enable);

  /** Set the current limits for the drive motor. */
  public void setCurrentLimits(double supplyLimit);

  /** Set the raw output for the drive motor. */
  public void setDriveRawOutput(double output);

  /** Set the raw output for the turn motor. */
  public void setTurnRawOutput(double output);

  /** Set the drive motor's PID gains. */
  public void setDrivePID(double kP, double kI, double kD);

  /** Set the turn motor's PID gains. */
  public void setTurnPID(double kP, double kI, double kD);

  /** Resets the turn motor's position. Useful for syncing up with the CanCoder */
  public void resetTurnMotor(Angle position);
}
