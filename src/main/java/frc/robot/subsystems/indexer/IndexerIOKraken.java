package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
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
  private TalonFX m_motor;

  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<Angle> m_motorPosition;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;

  private final TalonFXConfiguration m_config;

  private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  // private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(false);

  public IndexerIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);

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

    m_motor.getConfigurator().apply(m_config);

    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorPosition = m_motor.getPosition();
    m_motorVelocity = m_motor.getVelocity();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorTemperature = m_motor.getDeviceTemp();

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connectedMotor,
        m_motorPosition,
        m_motorVelocity,
        m_motorCurrent,
        m_motorStatorCurrent,
        m_motorVoltage,
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
  public void updateInputs(IndexerInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      System.out.println(
          BaseStatusSignal.refreshAll(
                  m_connectedMotor,
                  m_motorPosition,
                  m_motorVelocity,
                  m_motorCurrent,
                  m_motorStatorCurrent,
                  m_motorVoltage,
                  m_motorTemperature)
              .isOK());
    }

    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.position = m_motorPosition.getValueAsDouble();
    inputs.velocityRPS = m_motorVelocity.getValueAsDouble();
    inputs.current = m_motorCurrent.getValueAsDouble();
    inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
    inputs.voltage = m_motorVoltage.getValueAsDouble();
    inputs.temperature = m_motorTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setControl(m_voltageOut.withOutput(voltage));
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }
}
