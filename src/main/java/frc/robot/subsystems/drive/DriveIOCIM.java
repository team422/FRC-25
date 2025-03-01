package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class DriveIOCIM implements DriveIO {
  private SparkMax m_LF;
  private SparkMax m_LR;
  private SparkMax m_RF;
  private SparkMax m_RR;

  public DriveIOCIM() {
    m_LF = new SparkMax(1, MotorType.kBrushed);
    m_LR = new SparkMax(2, MotorType.kBrushed);
    m_RF = new SparkMax(4, MotorType.kBrushed);
    m_RR = new SparkMax(3, MotorType.kBrushed);
  }

  public SparkMax getMotor(int id) {
    switch (id) {
      case 1:
        return m_LF;
      case 2:
        return m_LR;
      case 4:
        return m_RF;
      case 3:
        return m_RR;
      default:
        return new SparkMax(0, MotorType.kBrushed);
    }
  }

  public void updateInputs(DriveInputs inputs) {
    for (int id = 0; id < 4; id++) {
      inputs.voltage[id] = this.getMotor(id).getBusVoltage();
      inputs.current[id] = this.getMotor(id).getOutputCurrent();
      inputs.output[id] = this.getMotor(id).getAppliedOutput();
    }
  }
}
