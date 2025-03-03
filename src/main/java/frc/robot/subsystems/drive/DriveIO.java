package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public class DriveInputs {
    public double[] voltage = {0, 0, 0, 0};
    public double[] current = {0, 0, 0, 0};
  }

  public void updateInputs(DriveInputs inputs);

  public void setLeft(double value);

  public void setRight(double value);
}
