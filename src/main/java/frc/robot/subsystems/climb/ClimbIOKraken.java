package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.Ports;

public class ClimbIOKraken implements ClimbIO {
  private TalonFX m_motor;

  private final TalonFXConfiguration m_config;
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);

  // Status Signals
  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<Angle> m_motorPosition;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Temperature> m_motorTemperature;

  public ClimbIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kDriveCanivoreName);

    var currentLimitConfigs =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kClimbDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kClimbDefaultStatorLimit);

    var feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(ClimbConstants.kClimbReduction);

    var motorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutput);

    m_motor.getConfigurator().apply(m_config);

    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorPosition = m_motor.getPosition();
    m_motorTemperature = m_motor.getDeviceTemp();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();

    // higher frequency for position
    BaseStatusSignal.setUpdateFrequencyForAll(100, m_motorPosition);

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75,
        m_connectedMotor,
        m_motorTemperature,
        m_motorVoltage,
        m_motorCurrent,
        m_motorStatorCurrent);

    ParentDevice.optimizeBusUtilizationForAll(m_motor);
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_connectedMotor,
          m_motorPosition,
          m_motorVoltage,
          m_motorCurrent,
          m_motorStatorCurrent,
          m_motorTemperature);
    }

    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.currPositionDegrees = getCurrPosition().getDegrees();
    inputs.desiredPositionDegrees = m_desiredAngle.getDegrees();
    inputs.atSetpoint = atSetpoint();
    inputs.voltage = m_motorVoltage.getValueAsDouble();
    inputs.current = m_motorCurrent.getValueAsDouble();
    inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
    inputs.temperature = m_motorTemperature.getValueAsDouble();
  }

  @Override
  public void setPID(int slot, double kP, double kI, double kD) {
    var slotConfigs = new SlotConfigs().withKP(kP).withKI(kI).withKD(kD);
    slotConfigs.SlotNumber = slot;
    m_motor.getConfigurator().apply(slotConfigs, 0.0);
  }

  @Override
  public void setSlot(int slot) {
    m_positionVoltage = m_positionVoltage.withSlot(slot);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle, double feedforward) {
    double value = angle.getRadians();
    value =
        MathUtil.clamp(
            value, ClimbConstants.kMinAngle.getRadians(), ClimbConstants.kMaxAngle.getRadians());
    angle = Rotation2d.fromRadians(value);

    m_desiredAngle = angle;
    m_motor.setControl(
        m_positionVoltage.withPosition(angle.getMeasure()).withFeedForward(feedforward));
  }

  @Override
  public void zeroEncoder() {
    m_motor.setPosition(0, 0.0);
  }

  public Rotation2d getCurrPosition() {
    return Rotation2d.fromRotations(m_motorPosition.getValueAsDouble());
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredAngle.getDegrees() - getCurrPosition().getDegrees())
        < ClimbConstants.kClimbTolerance;
  }
}
