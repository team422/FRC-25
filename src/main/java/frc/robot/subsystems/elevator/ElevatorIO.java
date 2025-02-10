package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputs {
    public boolean isLeadingMotorConnected;
    public boolean isFollowingMotorConnected;
    public double desiredLocation;
    public boolean atSetpoint;
    public double leadingPosition;
    public double followingPosition;
    public double leadingVelocity;
    public double followingVelocity;
    public double leadingAcceleration;
    public double followingAcceleration;
    public double leadingVoltage;
    public double followingVoltage;
    public double leadingSupplyCurrent;
    public double followingSupplyCurrent;
    public double leadingStatorCurrent;
    public double followingStatorCurrent;
    public double leadingTemp;
    public double followingTemp;
  }

  public void updateInputs(ElevatorInputs inputs);

  public void setDesiredHeight(double inchesHeight);

  public void setPIDFF(
      int slot, double kP, double kI, double kD, double kS, double kV, double kA, double kG);

  public void setSlot(int slot);

  public void setCurrentLimits(double supplyLimit);

  public void setMagic(double velocity, double acceleration, double jerk);

  public boolean atSetpoint();

  public double getCurrHeight();
}
