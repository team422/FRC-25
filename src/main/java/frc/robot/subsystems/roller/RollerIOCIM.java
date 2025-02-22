package frc.robot.subsystems.roller;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class RollerIOCIM implements RollerIO{
    private SparkMax m_motor;

    public RollerIOCIM() {
        m_motor = new SparkMax(0, MotorType.kBrushed);
        
    }

    public void updateInputs(RollerInputs inputs) {
        inputs.voltage = m_motor.getBusVoltage();
        inputs.current = m_motor.getOutputCurrent();
        inputs.velocity = m_motor.getAppliedOutput();
    }

    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }
}
