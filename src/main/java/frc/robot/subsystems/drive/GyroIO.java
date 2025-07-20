package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public double[] odometryTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public Rotation2d[] odometryPitchPositions = new Rotation2d[] {};
    public Rotation2d[] odometryRollPositions = new Rotation2d[] {};
    public double yawVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double xAcceleration = 0.0;
    public double yAcceleration = 0.0;
    public double zAcceleration = 0.0;
  }

  public void updateInputs(GyroIOInputs inputs);
}
