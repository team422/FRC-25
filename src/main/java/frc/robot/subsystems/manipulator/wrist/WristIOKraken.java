package frc.robot.subsystems.manipulator.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;

public class WristIOKraken implements WristIO {
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

  // private PositionTorqueCurrentFOC m_positionControl =
  //     new PositionTorqueCurrentFOC(0.0).withSlot(0);
  private PositionVoltage m_positionControl =
      new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);
  private Rotation2d m_desiredAngle = new Rotation2d();

  public WristIOKraken(int port, int absoluteEncoderPort) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);

    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kManipulatorWristDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kManipulatorWristDefaultStatorLimit);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration().withCurrentLimits(currentLimits).withMotorOutput(motorOutput);

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
              m_motorVoltage,
              m_motorCurrent,
              m_motorStatorCurrent,
              m_motorTemperature));
    }
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_motorPosition,
          m_motorVelocity,
          m_motorVoltage,
          m_motorCurrent,
          m_motorStatorCurrent,
          m_motorTemperature);
    }

    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    // wait until the absolute encoder is actually giving a reading
    if (!m_relativeEncoderReset && m_absoluteEncoder.get() != 1) {
      m_relativeEncoderReset = true;
      resetRelativeEncoder();
    }

    inputs.currAngleDeg = getCurrAngle().getDegrees();
    inputs.desiredAngleDeg = m_desiredAngle.getDegrees();
    inputs.atSetpoint = atSetpoint();
    inputs.velocityRPS = m_motorVelocity.getValueAsDouble();
    inputs.current = m_motorCurrent.getValueAsDouble();
    inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
    inputs.voltage = m_motorVoltage.getValueAsDouble();
    inputs.temperature = m_motorTemperature.getValueAsDouble();
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kG) {
    m_motor
        .getConfigurator()
        .apply(
            m_config.Slot0.withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKG(kG)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign),
            0.0);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    m_desiredAngle = angle;
    m_motor.setControl(m_positionControl.withPosition(angle.getRotations()));
  }

  @Override
  public Rotation2d getCurrAngle() {
    return Rotation2d.fromRotations(m_motorPosition.getValueAsDouble());
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(m_desiredAngle.getDegrees() - getCurrAngle().getDegrees())
        < ManipulatorConstants.kWristTolerance;
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
    double offset =
        ManipulatorConstants.kWristOffset.getRotations()
            + m_absoluteEncoder.get()
            - getCurrAngle().getRotations();
    m_config.Feedback.withFeedbackRotorOffset(offset)
        .withSensorToMechanismRatio(ManipulatorConstants.kWristGearRatio);
    m_motor.getConfigurator().apply(m_config.Feedback, 0.0);
  }
}
