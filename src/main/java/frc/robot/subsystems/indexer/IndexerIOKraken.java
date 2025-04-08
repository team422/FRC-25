package frc.robot.subsystems.indexer;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;

public class IndexerIOKraken implements IndexerIO {
  private TalonFX m_sideMotor;
  private TalonFX m_topMotor;

  private StatusSignal<ConnectedMotorValue> m_sideConnectedMotor;
  private StatusSignal<Angle> m_sideMotorPosition;
  private StatusSignal<AngularVelocity> m_sideMotorVelocity;
  private StatusSignal<Current> m_sideMotorCurrent;
  private StatusSignal<Current> m_sideMotorStatorCurrent;
  private StatusSignal<Voltage> m_sideMotorVoltage;
  private StatusSignal<Temperature> m_sideMotorTemperature;

  private StatusSignal<ConnectedMotorValue> m_topConnectedMotor;
  private StatusSignal<Angle> m_topMotorPosition;
  private StatusSignal<AngularVelocity> m_topMotorVelocity;
  private StatusSignal<Current> m_topMotorCurrent;
  private StatusSignal<Current> m_topMotorStatorCurrent;
  private StatusSignal<Voltage> m_topMotorVoltage;
  private StatusSignal<Temperature> m_topMotorTemperature;

  private final TalonFXConfiguration m_config;

  private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  // private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(false);

  public IndexerIOKraken(int sidePort, int topPort) {
    m_sideMotor = new TalonFX(sidePort, Ports.kMainCanivoreName);
    m_topMotor = new TalonFX(topPort, Ports.kDriveCanivoreName);

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

    m_sideConnectedMotor = m_sideMotor.getConnectedMotor();
    m_sideMotorPosition = m_sideMotor.getPosition();
    m_sideMotorVelocity = m_sideMotor.getVelocity();
    m_sideMotorCurrent = m_sideMotor.getSupplyCurrent();
    m_sideMotorStatorCurrent = m_sideMotor.getStatorCurrent();
    m_sideMotorVoltage = m_sideMotor.getMotorVoltage();
    m_sideMotorTemperature = m_sideMotor.getDeviceTemp();

    m_topConnectedMotor = m_topMotor.getConnectedMotor();
    m_topMotorPosition = m_topMotor.getPosition();
    m_topMotorVelocity = m_topMotor.getVelocity();
    m_topMotorCurrent = m_topMotor.getSupplyCurrent();
    m_topMotorStatorCurrent = m_topMotor.getStatorCurrent();
    m_topMotorVoltage = m_topMotor.getMotorVoltage();
    m_topMotorTemperature = m_topMotor.getDeviceTemp();

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_sideConnectedMotor,
        m_sideMotorPosition,
        m_sideMotorVelocity,
        m_sideMotorCurrent,
        m_sideMotorStatorCurrent,
        m_sideMotorVoltage,
        m_sideMotorTemperature,
        m_topConnectedMotor,
        m_topMotorPosition,
        m_topMotorVelocity,
        m_topMotorCurrent,
        m_topMotorStatorCurrent,
        m_topMotorVoltage,
        m_topMotorTemperature);

    // ParentDevice.optimizeBusUtilizationForAll(m_sideMotor, m_topMotor);

    if (Constants.kUseBaseRefreshManager) {
      CtreBaseRefreshManager.addSignals(
          List.of(
              m_sideConnectedMotor,
              m_sideMotorPosition,
              m_sideMotorVelocity,
              m_sideMotorCurrent,
              m_sideMotorStatorCurrent,
              m_sideMotorVoltage,
              m_sideMotorTemperature,
              m_topConnectedMotor,
              m_topMotorPosition,
              m_topMotorVelocity,
              m_topMotorCurrent,
              m_topMotorStatorCurrent,
              m_topMotorVoltage,
              m_topMotorTemperature));
    }

    // m_topMotor.setControl(new Follower(sidePort, false));
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
              m_sideConnectedMotor,
              m_sideMotorPosition,
              m_sideMotorVelocity,
              m_sideMotorCurrent,
              m_sideMotorStatorCurrent,
              m_sideMotorVoltage,
              m_sideMotorTemperature,
              m_topConnectedMotor,
              m_topMotorPosition,
              m_topMotorVelocity,
              m_topMotorCurrent,
              m_topMotorStatorCurrent,
              m_topMotorVoltage,
              m_topMotorTemperature)
          .isOK();
    }

    inputs.sideMotorIsConnected = m_sideConnectedMotor.getValue() != ConnectedMotorValue.Unknown;
    inputs.topMotorIsConnected = m_topConnectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.sidePosition = m_sideMotorPosition.getValueAsDouble();
    inputs.sideVelocityRPS = m_sideMotorVelocity.getValueAsDouble();
    inputs.sideCurrent = m_sideMotorCurrent.getValueAsDouble();
    inputs.sideStatorCurrent = m_sideMotorStatorCurrent.getValueAsDouble();
    inputs.sideVoltage = m_sideMotorVoltage.getValueAsDouble();
    inputs.sideTemperature = m_sideMotorTemperature.getValueAsDouble();

    inputs.topPosition = m_topMotorPosition.getValueAsDouble();
    inputs.topVelocityRPS = m_topMotorVelocity.getValueAsDouble();
    inputs.topCurrent = m_topMotorCurrent.getValueAsDouble();
    inputs.topStatorCurrent = m_topMotorStatorCurrent.getValueAsDouble();
    inputs.topVoltage = m_topMotorVoltage.getValueAsDouble();
    inputs.topTemperature = m_topMotorTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double sideVoltage, double topVoltage) {
    m_sideMotor.setControl(m_voltageOut.withOutput(sideVoltage));
    m_topMotor.setControl(m_voltageOut.withOutput(topVoltage));
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_sideMotor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
    m_topMotor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }
}
