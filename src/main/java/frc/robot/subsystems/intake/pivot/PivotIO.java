package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotInputs {
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

  public void updateInputs(PivotInputs inputs);

  public void setPIDFF(int slot, double kP, double kI, double kD, double kS);

  public void setDesiredAngle(Rotation2d angle, double feedforward);

  public void setCurrentLimits(double supplyLimit);

  public void setSlot(int slot);

  public void zeroEncoder(Angle value);
}
