package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.Ports;

public class IntakeIOKraken implements IntakeIO {
  private TalonFX m_topMotor;
  private TalonFX m_sideMotor;
  private TalonFXConfiguration m_config;

  private VoltageOut m_topVoltage = new VoltageOut(0).withEnableFOC(true);
  private VoltageOut m_sideVoltage = new VoltageOut(0).withEnableFOC(true);

  private StatusSignal<ConnectedMotorValue> m_topConnected;
  private StatusSignal<ConnectedMotorValue> m_sideConnected;
  private StatusSignal<Voltage> m_topVoltageSignal;
  private StatusSignal<Voltage> m_sideVoltageSignal;
  private StatusSignal<AngularVelocity> m_topVelocity;
  private StatusSignal<AngularVelocity> m_sideVelocity;
  private StatusSignal<Current> m_topSupplyCurrent;
  private StatusSignal<Current> m_sideSupplyCurrent;
  private StatusSignal<Current> m_topStatorCurrent;
  private StatusSignal<Current> m_sideStatorCurrent;
  private StatusSignal<Temperature> m_topTemperature;
  private StatusSignal<Temperature> m_sideTemperature;

  public IntakeIOKraken(int top, int side) {
    m_topMotor = new TalonFX(top, Ports.kMainCanivoreName);
    m_sideMotor = new TalonFX(side, Ports.kMainCanivoreName);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIndexerDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kIndexerDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(IndexerConstants.kGearRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    m_sideMotor.getConfigurator().apply(m_config);
    m_topMotor.getConfigurator().apply(m_config);

    m_topConnected = m_topMotor.getConnectedMotor();
    m_sideConnected = m_sideMotor.getConnectedMotor();

    m_topVoltageSignal = m_topMotor.getMotorVoltage();
    m_sideVoltageSignal = m_sideMotor.getMotorVoltage();

    m_topVelocity = m_topMotor.getVelocity();
    m_sideVelocity = m_sideMotor.getVelocity();

    m_topSupplyCurrent = m_topMotor.getSupplyCurrent();
    m_sideSupplyCurrent = m_sideMotor.getSupplyCurrent();

    m_topStatorCurrent = m_topMotor.getStatorCurrent();
    m_sideStatorCurrent = m_sideMotor.getStatorCurrent();

    m_topTemperature = m_topMotor.getDeviceTemp();
    m_sideTemperature = m_sideMotor.getDeviceTemp();
    StatusSignal.setUpdateFrequencyForAll(
        50,
        m_topConnected,
        m_sideConnected,
        m_topVoltageSignal,
        m_sideVoltageSignal,
        m_topVelocity,
        m_sideVelocity,
        m_topSupplyCurrent,
        m_sideSupplyCurrent,
        m_topStatorCurrent,
        m_sideStatorCurrent,
        m_topTemperature,
        m_sideTemperature);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_topConnected,
        m_sideConnected,
        m_topVoltageSignal,
        m_sideVoltageSignal,
        m_topVelocity,
        m_sideVelocity,
        m_topSupplyCurrent,
        m_sideSupplyCurrent,
        m_topStatorCurrent,
        m_sideStatorCurrent,
        m_topTemperature,
        m_sideTemperature);

    inputs.topConnected = m_topConnected.getValue() != ConnectedMotorValue.Unknown;
    inputs.sideConnected = m_sideConnected.getValue() != ConnectedMotorValue.Unknown;

    inputs.topVoltage = m_topVoltageSignal.getValueAsDouble();
    inputs.sideVoltage = m_sideVoltageSignal.getValueAsDouble();

    inputs.topVelocity = m_topVelocity.getValueAsDouble();
    inputs.sideVelocity = m_sideVelocity.getValueAsDouble();

    inputs.topSupplyCurrent = m_topSupplyCurrent.getValueAsDouble();
    inputs.sideSupplyCurrent = m_sideSupplyCurrent.getValueAsDouble();

    inputs.topStatorCurrent = m_topStatorCurrent.getValueAsDouble();
    inputs.sideStatorCurrent = m_sideStatorCurrent.getValueAsDouble();

    inputs.topTemperature = m_topTemperature.getValueAsDouble();
    inputs.bottomTemperature = m_sideTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double top, double side) {
    m_topMotor.setControl(m_topVoltage.withOutput(top));
    m_sideMotor.setControl(m_sideVoltage.withOutput(side));
  }
}
