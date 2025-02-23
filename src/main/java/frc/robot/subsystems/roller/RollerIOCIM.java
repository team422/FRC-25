package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class RollerIOCIM implements RollerIO {
  private SparkMax m_motor;

  public RollerIOCIM() {
    m_motor = new SparkMax(5, MotorType.kBrushed);
  }

  public void updateInputs(RollerInputs inputs) {
    inputs.voltage = m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();
    inputs.output = m_motor.getAppliedOutput();
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
