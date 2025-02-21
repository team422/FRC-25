package frc.robot.subsystems.intake.roller;

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
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;

public class IntakeRollerIOKraken implements IntakeRollerIO {
  private TalonFX m_motor;

  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<AngularAcceleration> m_motorAcceleration;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;

  private final TalonFXConfiguration m_config;

  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public IntakeRollerIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kDriveCanivoreName);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeRollerDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeRollerDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kRollerGearRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    m_motor.getConfigurator().apply(m_config);

    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorVelocity = m_motor.getVelocity();
    m_motorAcceleration = m_motor.getAcceleration();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorTemperature = m_motor.getDeviceTemp();

    // acceleration is important for some of our logic so higher frequency
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, m_motorAcceleration);

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
              m_motorVelocity,
              m_motorAcceleration,
              m_motorVoltage,
              m_motorCurrent,
              m_motorStatorCurrent,
              m_motorTemperature));
    }
  }

  @Override
  public void updateInputs(IntakeRollerInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_motorVelocity,
          m_motorAcceleration,
          m_motorVoltage,
          m_motorCurrent,
          m_motorStatorCurrent,
          m_motorTemperature);
    }

    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.velocityRPS = m_motorVelocity.getValueAsDouble();
    inputs.accelerationRPSSq = m_motorAcceleration.getValueAsDouble();
    inputs.voltage = m_motorVoltage.getValueAsDouble();
    inputs.current = m_motorCurrent.getValueAsDouble();
    inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
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

  @Override
  public boolean hasGamePiece() {
    double current = m_motorCurrent.getValueAsDouble();
    double accel = m_motorAcceleration.getValueAsDouble();
    return current > IntakeConstants.kRollerCurrentGamepieceThreshold
        && accel < IntakeConstants.kRollerAccelGamepieceThreshold;
  }
}
