package frc.robot.subsystems.manipulator.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.Ports;
import org.littletonrobotics.junction.Logger;

public class WristIOKraken implements WristIO {
  private TalonFX m_motor;
  private DutyCycleEncoder m_absoluteEncoder;
  private boolean m_reset;
  private TalonFXConfiguration m_configs;
  private Rotation2d m_desired;
  private PositionVoltage m_voltage = new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);

  private StatusSignal<Angle> m_position;
  private StatusSignal<Voltage> m_voltageSignal;
  private StatusSignal<AngularVelocity> m_velocity;
  private StatusSignal<ConnectedMotorValue> m_connected;
  private StatusSignal<Current> m_supplyCurrent;
  private StatusSignal<Current> m_statorCurrent;
  private StatusSignal<Temperature> m_temperature;

  public WristIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);
    m_motor.setPosition(Rotation2d.fromDegrees(130).getRotations());

    m_absoluteEncoder = new DutyCycleEncoder(Ports.kManipulatorAbsoluteEncoder);
    m_reset = false;

    m_desired = new Rotation2d();

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kManipulatorWristDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kManipulatorWristDefaultStatorLimit);

    var motorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    var feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(ManipulatorConstants.kWristGearRatio);

    m_configs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withMotorOutput(motorOutput)
            .withFeedback(feedback);

    m_motor.getConfigurator().apply(m_configs);

    m_position = m_motor.getPosition();
    m_voltageSignal = m_motor.getMotorVoltage();
    m_velocity = m_motor.getVelocity();
    m_connected = m_motor.getConnectedMotor();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_temperature = m_motor.getDeviceTemp();

    StatusSignal.setUpdateFrequencyForAll(
        100,
        m_position,
        m_voltageSignal,
        m_velocity,
        m_connected,
        m_supplyCurrent,
        m_statorCurrent,
        m_temperature);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.atSetpoint =
        Math.abs(
                Rotation2d.fromRotations(m_motor.getPosition().getValueAsDouble()).getDegrees()
                    - m_desired.getDegrees())
            < ManipulatorConstants.kWristTolerance;
    inputs.connected = m_connected.getValue() != ConnectedMotorValue.Unknown;
    inputs.desired = m_desired.getDegrees();
    Logger.recordOutput("Elevator/here", m_desired.getDegrees());
    inputs.position = Rotation2d.fromRotations(m_position.getValueAsDouble()).getDegrees();
    inputs.statorCurrent = m_statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = m_supplyCurrent.getValueAsDouble();
    inputs.temperature = m_temperature.getValueAsDouble();
    inputs.velocity = m_velocity.getValueAsDouble();
    inputs.voltage = m_voltageSignal.getValueAsDouble();

    if (!m_reset && m_absoluteEncoder.get() != 0) {
      m_reset = true;
      zero();
    }
  }

  @Override
  public void setAngle(Rotation2d angle) {
    Logger.recordOutput("Elevator/nikhil", angle.getDegrees());
    Logger.recordOutput("Elevator/kill", m_desired.getDegrees());
    this.m_desired = angle;
    Logger.recordOutput("Elevator/hate", angle.getDegrees());
    Logger.recordOutput("Elevator/curr", m_desired.getDegrees());
    m_motor.setControl(m_voltage.withPosition(angle.getRotations()));
  }

  @Override
  public void setPIDFF(int slot, double p, double i, double d, double kS) {
    var config = new SlotConfigs().withKP(p).withKI(i).withKD(d).withKS(kS);
    config.SlotNumber = 0;

    m_motor.getConfigurator().apply(config);
  }

  private void zero() {
    double raw = m_absoluteEncoder.get();
    raw += ManipulatorConstants.kWristOffset.getRotations();
    raw %= 1;
    if (raw < 0) {
      raw += 1;
    }
    double rotations =
        Rotation2d.fromRotations(raw)
            .div(ManipulatorConstants.kWristAbsoluteEncoderGearRatio)
            .plus(Rotation2d.fromDegrees(23))
            .getRotations();
    m_motor.setPosition(rotations);
  }
}
