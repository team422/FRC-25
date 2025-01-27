package frc.robot.subsystems.manipulator.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorRollerIOKraken implements ManipulatorRollerIO {
  private TalonFX m_motor;

  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;

  private final TalonFXConfiguration m_config;

  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public ManipulatorRollerIOKraken(int port) {
    m_motor = new TalonFX(port);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kManipulatorRollerDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kManipulatorRollerDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ManipulatorConstants.kRollerGearRatio);

    m_config =
        new TalonFXConfiguration().withCurrentLimits(currentLimits).withFeedback(feedbackConfig);

    m_motor.getConfigurator().apply(m_config);
    m_motor.setNeutralMode(NeutralModeValue.Brake);

    m_motorVelocity = m_motor.getVelocity();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorTemperature = m_motor.getDeviceTemp();

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_motorVelocity,
        m_motorCurrent,
        m_motorStatorCurrent,
        m_motorVoltage,
        m_motorTemperature);
    ParentDevice.optimizeBusUtilizationForAll(m_motor);
  }

  @Override
  public void updateInputs(ManipulatorRollerInputs inputs) {
    inputs.motorIsConnected =
        BaseStatusSignal.refreshAll(
                m_motorVelocity,
                m_motorCurrent,
                m_motorStatorCurrent,
                m_motorVoltage,
                m_motorTemperature)
            .isOK();

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
