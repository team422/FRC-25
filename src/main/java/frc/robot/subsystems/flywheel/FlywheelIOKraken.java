package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOKraken implements FlywheelIO {
  private TalonFX m_motor;

  private Slot0Configs m_slot0Config = new Slot0Configs();
  private VoltageOut m_voltageOut = new VoltageOut(0);
  private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  private double m_desiredVelocity = 0;
  private boolean m_velocityControl = false;

  public FlywheelIOKraken(int port) {
    m_motor = new TalonFX(port);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 30.0;

    m_motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.currVelocityRPS = m_motor.getVelocity().getValueAsDouble();
    inputs.desiredVelocityRPS = m_desiredVelocity;
    inputs.atSetpoint =
        Math.abs(m_desiredVelocity - inputs.currVelocityRPS) < FlywheelConstants.kVelocityTolerance;
    inputs.velocityControl = m_velocityControl;
    inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.current = m_motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kV) {
    m_slot0Config.kP = kP;
    m_slot0Config.kI = kI;
    m_slot0Config.kD = kD;
    m_slot0Config.kS = kS;
    m_slot0Config.kV = kV;

    m_motor.getConfigurator().apply(m_slot0Config);
  }

  @Override
  public void setDesiredVelocity(double velocityRPS) {
    m_velocityControl = true;
    m_desiredVelocity = velocityRPS;
    m_motor.setControl(m_velocityVoltage.withVelocity(velocityRPS));
  }

  @Override
  public void setVoltage(double voltage) {
    m_velocityControl = false;
    m_desiredVelocity = 0;
    m_motor.setControl(m_voltageOut.withOutput(voltage));
  }
}
