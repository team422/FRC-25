package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbInputs {
    public double currPositionRad;
    public double desiredPositionRad;
    public boolean atSetpoint;
    public boolean positionControl;
    public double voltage;
    public double current;
  }

  public void updateInputs(ClimbInputs inputs);

  public void setPIDFF(double kP, double kI, double kD, double kS, double kV);

  public void setVoltage(double voltage);

  public void setDesiredAngle(Rotation2d angle);
}
