package frc.robot.subsystems.manipulator.rollers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Ports;

public class RollerIOKraken implements RollerIO {
  private TalonFX m_motor;
  private StatusSignal<AngularVelocity> m_velocitySignal;
  private StatusSignal<Voltage> m_voltageSignal;
  private StatusSignal<ConnectedMotorValue> m_connected;
  private StatusSignal<Current> m_supplyCurrent;
  private StatusSignal<Current> m_statorCurrent;
  private StatusSignal<Temperature> m_temperature;

  public RollerIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);

    m_velocitySignal = m_motor.getVelocity();
    m_voltageSignal = m_motor.getMotorVoltage();
    m_connected = m_motor.getConnectedMotor();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_temperature = m_motor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        100,
        m_velocitySignal,
        m_voltageSignal,
        m_connected,
        m_supplyCurrent,
        m_statorCurrent,
        m_temperature);
  }

  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }

  public void updateInputs(RollerInputs inputs) {
    inputs.velocity = m_velocitySignal.getValueAsDouble();
    inputs.voltage = m_voltageSignal.getValueAsDouble();
    inputs.connected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    inputs.supplyCurrent = m_supplyCurrent.getValueAsDouble();
    inputs.statorCurrent = m_statorCurrent.getValueAsDouble();
    inputs.temperature = m_temperature.getValueAsDouble();
  }
}
