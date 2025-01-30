package frc.robot.subsystems.manipulator.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristInputs {
    public double currAngleDeg;
    public double desiredAngleDeg;
    public boolean atSetpoint;
    public double velocityRPS;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(WristInputs inputs);

  public void setPIDFF(double kP, double kI, double kD, double kS, double kG);

  public void setDesiredAngle(Rotation2d angle);

  public Rotation2d getCurrAngle();

  public boolean atSetpoint();

  public void setCurrentLimits(double supplyLimit);
}
