package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class DriveIOCIM implements DriveIO {
  private SparkMax m_LF;
  private SparkMax m_LR;
  private SparkMax m_RF;
  private SparkMax m_RR;

  public DriveIOCIM() {
    m_LF = new SparkMax(0, MotorType.kBrushed);
    m_LR = new SparkMax(1, MotorType.kBrushed);
    m_RF = new SparkMax(2, MotorType.kBrushed);
    m_RR = new SparkMax(3, MotorType.kBrushed);
  }

  public void updateInputs(DriveInputs inputs) {
    inputs.LF =
        new double[] {m_LF.getBusVoltage(), m_LF.getOutputCurrent(), m_LF.getAppliedOutput()};
    inputs.LR =
        new double[] {m_LR.getBusVoltage(), m_LR.getOutputCurrent(), m_LR.getAppliedOutput()};
    inputs.RF =
        new double[] {m_RF.getBusVoltage(), m_RF.getOutputCurrent(), m_RF.getAppliedOutput()};
    inputs.RR =
        new double[] {m_RR.getBusVoltage(), m_RR.getOutputCurrent(), m_RR.getAppliedOutput()};
  }

  public void setVoltage(double voltageLF, double voltageLR, double voltageRF, double voltageRR) {
    m_LF.setVoltage(voltageLF);
    m_LR.setVoltage(voltageLR);
    m_RF.setVoltage(voltageRF);
    m_RR.setVoltage(voltageRR);
  }
}
