package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputsAutoLogged {
    public double currLocation;
    public double desiredLocation;
    public boolean atSetpoint;
    public double voltage;
  }

  public void updateInputs(ElevatorInputsAutoLogged inputs);

  public void setDesiredHeight(double centimeters);

  public void setPIDFF(double kP, double kI, double kD, double kS, double kV, double kA, double kG);
}
