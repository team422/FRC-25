package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbInputs {
    public double currPositionDegrees;
    public double desiredPositionDegrees;
    public boolean atSetpoint;
    public double voltage;
    public double current;
    public double statorCurrent;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(ClimbInputs inputs);

  public void setPID(int slot, double kP, double kI, double kD);

  public void setSlot(int slot);

  public void setDesiredAngle(Rotation2d angle, double feedforward);

  public void zeroEncoder();

  public boolean atSetpoint();
}
