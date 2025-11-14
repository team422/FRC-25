package frc.robot.subsystems.manipulator.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public class WristInputs {
    public double position;
    public double desired;
    public double voltage;
    public double velocity;
    public boolean connected;
    public double supplyCurrent;
    public double statorCurrent;
    public double temperature;
    public boolean atSetpoint;
  }

  public void updateInputs(WristInputs inputs);

  public void setAngle(Rotation2d angle);

  public void setPIDFF(int slot, double p, double i, double d, double kS);
}
