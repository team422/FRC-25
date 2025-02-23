package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public class DriveInputs {
    public double[] voltage = {0, 0, 0, 0};
    public double[] current = {0, 0, 0, 0};
    public double[] output = {0, 0, 0, 0};
  }

  public void updateInputs(DriveInputs inputs);

  public SparkMax getMotor(int id);
}
