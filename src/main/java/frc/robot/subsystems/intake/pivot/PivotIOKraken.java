package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;

public class PivotIOKraken implements PivotIO {
  private TalonFX m_motor;

  private DutyCycleEncoder m_absoluteEncoder;

  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<Angle> m_motorPosition;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;

  private final TalonFXConfiguration m_config;

  private boolean m_relativeEncoderReset = false;

  private PositionVoltage m_positionControl = new PositionVoltage(0.0).withSlot(0);
  private Rotation2d m_desiredAngle = new Rotation2d();

  public PivotIOKraken(int port, int absoluteEncoderPort) {
    m_motor = new TalonFX(port, Ports.kDriveCanivoreName);

    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakePivotDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakePivotDefaultStatorLimit);

    var motorOutput =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

    var feedback =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kPivotGearRatio);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withMotorOutput(motorOutput)
            .withFeedback(feedback);

    m_motor.getConfigurator().apply(m_config);

    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorPosition = m_motor.getPosition();
    m_motorVelocity = m_motor.getVelocity();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorTemperature = m_motor.getDeviceTemp();

    // higher frequency for position
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, m_motorPosition);

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connectedMotor,
        m_motorVelocity,
        m_motorVoltage,
        m_motorCurrent,
        m_motorStatorCurrent,
        m_motorTemperature);

    ParentDevice.optimizeBusUtilizationForAll(m_motor);

    if (Constants.kUseBaseRefreshManager) {
      CtreBaseRefreshManager.addSignals(
          List.of(
              m_connectedMotor,
              m_motorPosition,
              m_motorVelocity,
              m_motorCurrent,
              m_motorStatorCurrent,
              m_motorVoltage,
              m_motorTemperature));
    }
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_motorPosition,
          m_motorVelocity,
          m_motorVoltage,
          m_motorCurrent,
          m_motorStatorCurrent,
          m_motorTemperature);
    }

    // wait until the absolute encoder is actually giving a reading
    if (!m_relativeEncoderReset && m_absoluteEncoder.get() != 1) {
      m_relativeEncoderReset = true;
      resetRelativeEncoder();
    }

    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.currAngleDeg = getCurrAngle().getDegrees();
    inputs.desiredAngleDeg = m_desiredAngle.getDegrees();
    inputs.atSetpoint = atSetpoint();
    inputs.velocityRPS = m_motorVelocity.getValue().in(RotationsPerSecond);
    inputs.current = m_motorCurrent.getValue().in(Amps);
    inputs.statorCurrent = m_motorStatorCurrent.getValue().in(Amps);
    inputs.voltage = m_motorVoltage.getValue().in(Volts);
    inputs.temperature = m_motorTemperature.getValue().in(Celsius);
  }

  @Override
  public void setPIDFF(int slot, double kP, double kI, double kD, double kS) {
    var slotConfigs =
        new SlotConfigs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    slotConfigs.SlotNumber = slot;
    m_motor.getConfigurator().apply(slotConfigs, 0.0);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle, double feedforward) {
    double value = angle.getRadians();
    value =
        MathUtil.clamp(
            value,
            IntakeConstants.kPivotMinAngle.getRadians(),
            IntakeConstants.kPivotMaxAngle.getRadians());
    angle = Rotation2d.fromRadians(value);

    m_desiredAngle = angle;
    m_motor.setControl(
        m_positionControl.withPosition(angle.getRotations()).withFeedForward(feedforward));
  }

  private Rotation2d getCurrAngle() {
    return new Rotation2d(m_motorPosition.getValue());
  }

  private boolean atSetpoint() {
    return Math.abs(m_desiredAngle.getDegrees() - getCurrAngle().getDegrees())
        < IntakeConstants.kPivotTolerance.get();
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }

  private void resetRelativeEncoder() {
    // for performance, we use the absolute encoder to set the start angle but rely on the relative
    // encoder for the rest of the time

    double startAngle = getAbsoluteFinal().getRotations();

    m_motor.getConfigurator().setPosition(startAngle);
  }

  private Rotation2d getAbsoluteWrapAround() {
    double rawValue = m_absoluteEncoder.get();
    rawValue += IntakeConstants.kPivotOffset.getRotations();
    // fix wrap around after offset applied
    // 360 is the range of the absolute encoder before it wraps around
    rawValue %= Units.degreesToRotations(360);
    if (rawValue < 0) {
      rawValue += 1.0;
    }
    return Rotation2d.fromRotations(rawValue);
  }

  private Rotation2d getAbsoluteFinal() {
    return getAbsoluteWrapAround().div(IntakeConstants.kPivotAbsoluteEncoderGearRatio);
  }

  @Override
  public void setSlot(int slot) {
    m_positionControl.withSlot(slot);
  }

  @Override
  public void zeroEncoder(Angle value) {
    m_motor.getConfigurator().setPosition(value);
  }
}
