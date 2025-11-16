package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;

public class ElevatorIOKraken implements ElevatorIO {
  private TalonFX m_leadingMotor;
  private TalonFX m_followingMotor;
  private double m_desiredHeight;

  private TalonFXConfiguration m_config;
  private PositionVoltage m_positionVoltage =
      new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);

  private StatusSignal<Angle> m_leadingPosition;
  private StatusSignal<Angle> m_followingPosition;
  private StatusSignal<ConnectedMotorValue> m_leadingConnected;
  private StatusSignal<ConnectedMotorValue> m_followingConnected;
  private StatusSignal<Voltage> m_leadingVoltage;
  private StatusSignal<Voltage> m_followingVoltage;
  private StatusSignal<Current> m_leadingSupplyCurrent;
  private StatusSignal<Current> m_followingSupplyCurrent;
  private StatusSignal<Current> m_leadingStatorCurrent;
  private StatusSignal<Current> m_followingStatorCurrent;
  private StatusSignal<Temperature> m_leadingTemperature;
  private StatusSignal<Temperature> m_followingTemperature;
  private StatusSignal<AngularVelocity> m_leadingVelocity;
  private StatusSignal<AngularVelocity> m_followingVelocity;

  public ElevatorIOKraken(int leadingPort, int followingPort) {
    m_leadingMotor = new TalonFX(leadingPort, Ports.kMainCanivoreName);
    m_followingMotor = new TalonFX(followingPort, Ports.kMainCanivoreName);
    m_desiredHeight = 0.0;

    var currentConfigs =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kElevatorDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kElevatorDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.kSensorToMechanismRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
            .withFeedback(feedbackConfig)
            .withCurrentLimits(currentConfigs)
            .withMotorOutput(motorOutput);

    m_leadingMotor.getConfigurator().apply(m_config);
    m_followingMotor
        .getConfigurator()
        .apply(m_config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    m_leadingMotor.getConfigurator().setPosition(0.0);
    m_followingMotor.getConfigurator().setPosition(0.0);
    m_followingMotor.setControl(new Follower(leadingPort, true));

    m_followingPosition = m_followingMotor.getPosition();
    m_leadingPosition = m_leadingMotor.getPosition();
    m_leadingConnected = m_leadingMotor.getConnectedMotor();
    m_followingConnected = m_followingMotor.getConnectedMotor();
    m_leadingVoltage = m_leadingMotor.getMotorVoltage();
    m_followingVoltage = m_followingMotor.getMotorVoltage();
    m_leadingSupplyCurrent = m_leadingMotor.getSupplyCurrent();
    m_followingSupplyCurrent = m_followingMotor.getSupplyCurrent();
    m_leadingStatorCurrent = m_leadingMotor.getStatorCurrent();
    m_followingStatorCurrent = m_followingMotor.getStatorCurrent();
    m_leadingTemperature = m_leadingMotor.getDeviceTemp();
    m_followingTemperature = m_followingMotor.getDeviceTemp();
    m_leadingVelocity = m_leadingMotor.getVelocity();
    m_followingVelocity = m_followingMotor.getVelocity();
    StatusSignal.setUpdateFrequencyForAll(
        100,
        m_followingPosition,
        m_leadingPosition,
        m_followingConnected,
        m_leadingConnected,
        m_leadingVelocity,
        m_followingVelocity,
        m_leadingVoltage,
        m_followingVoltage,
        m_leadingSupplyCurrent,
        m_followingSupplyCurrent,
        m_leadingStatorCurrent,
        m_followingStatorCurrent,
        m_leadingTemperature,
        m_followingTemperature);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_followingPosition,
        m_leadingPosition,
        m_followingConnected,
        m_leadingConnected,
        m_leadingVelocity,
        m_followingVelocity,
        m_leadingVoltage,
        m_followingVoltage,
        m_leadingSupplyCurrent,
        m_followingSupplyCurrent,
        m_leadingStatorCurrent,
        m_followingStatorCurrent,
        m_leadingTemperature,
        m_followingTemperature);
    inputs.leadingHeight = m_leadingPosition.getValueAsDouble();
    inputs.followingHeight = m_followingPosition.getValueAsDouble();
    inputs.desiredHeight = m_desiredHeight;
    inputs.atSetpoint =
        Math.abs(inputs.leadingHeight - m_desiredHeight) < ElevatorConstants.kHeightTolerance
            && Math.abs(inputs.followingHeight - m_desiredHeight)
                < ElevatorConstants.kHeightTolerance;
    inputs.leadingMotorConnected = m_leadingConnected.getValue() != ConnectedMotorValue.Unknown;
    inputs.followingMotorConnected = m_followingConnected.getValue() != ConnectedMotorValue.Unknown;
    inputs.leadingVelocity = m_leadingVelocity.getValueAsDouble();
    inputs.followingVelocity = m_followingVelocity.getValueAsDouble();
    inputs.leadingVoltage = m_leadingVoltage.getValue().in(Volts);
    inputs.followingVoltage = m_followingVoltage.getValue().in(Volts);
    inputs.leadingSupplyCurrent = m_leadingSupplyCurrent.getValue().in(Amps);
    inputs.followingSupplyCurrent = m_followingSupplyCurrent.getValue().in(Amps);
    inputs.leadingStatorCurrent = m_leadingStatorCurrent.getValue().in(Amps);
    inputs.followingStatorCurrent = m_followingStatorCurrent.getValue().in(Amps);
    inputs.leadingTemperature = m_leadingTemperature.getValue().in(Celsius);
    inputs.followingTemperature = m_followingTemperature.getValue().in(Celsius);
  }

  @Override
  public void setHeight(double height) {
    height = MathUtil.clamp(height, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
    m_desiredHeight = height;

    m_followingMotor.setControl(m_positionVoltage.withPosition(height));
    m_leadingMotor.setControl(m_positionVoltage.withPosition(height));
  }

  @Override
  public void setPIDFF(int slot, double p, double i, double d, double kG) {
    // how i originally did it (to show progression ykwim)
    // var configs = m_config.Slot0;
    // configs.kP = p;
    // etc
    var configs = new SlotConfigs().withKP(p).withKI(i).withKD(d).withKG(kG);
    // wow so prettyyyyy
    configs.SlotNumber = slot;

    m_leadingMotor.getConfigurator().apply(configs);
    m_followingMotor.getConfigurator().apply(configs);
  }

  public void zero() {
    m_followingMotor.setPosition(0.0);
    m_leadingMotor.setPosition(0.0);
  }
}
