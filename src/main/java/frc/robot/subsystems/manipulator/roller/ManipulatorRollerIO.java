package frc.robot.subsystems.manipulator.roller;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorRollerIO {
  @AutoLog
  public static class ManipulatorRollerInputs {
    public double positionDegrees;
    public boolean positionControl;
    public double desiredPositionDegrees;
    public double velocityRPS;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(ManipulatorRollerInputs inputs);

  public void setVoltage(double voltage);

  // cannot use Rotation2d because this involves multiple rotations
  public void setDesiredPosition(Angle position);

  public Angle getPosition();

  public void setPositionPID(double kP, double kI, double kD);

  public void setCurrentLimits(double supplyLimit);
}
