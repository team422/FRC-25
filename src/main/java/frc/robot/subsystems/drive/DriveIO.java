package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public class DriveInputs {
    public double[] LF;
    public double[] LR;
    public double[] RF;
    public double[] RR;
  }

  public void updateInputs(DriveInputs inputs);

  public void setVoltage(double voltageLF, double voltageLR, double voltageRF, double voltageRR);
}
