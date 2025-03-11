package frc.robot.subsystems.manipulator.roller;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;

public class ManipulatorRollerIOKraken implements ManipulatorRollerIO {
  private TalonFX m_motor;

  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<Angle> m_motorPosition;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<AngularAcceleration> m_motorAcceleration;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Temperature> m_motorTemperature;

  private final TalonFXConfiguration m_config;

  private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  // private final VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(false);
  private final PositionVoltage m_positionVoltage =
      new PositionVoltage(0.0).withSlot(0).withEnableFOC(true);

  private boolean m_positionControl = false;
  private Angle m_desiredPosition = Degrees.of(0.0);

  public ManipulatorRollerIOKraken(int port) {
    m_motor = new TalonFX(port, Ports.kMainCanivoreName);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kManipulatorRollerDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kManipulatorRollerDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ManipulatorConstants.kRollerGearRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    m_motor.getConfigurator().apply(m_config);
    m_motor.getConfigurator().setPosition(0.0);

    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorPosition = m_motor.getPosition();
    m_motorVelocity = m_motor.getVelocity();
    m_motorAcceleration = m_motor.getAcceleration();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorTemperature = m_motor.getDeviceTemp();

    // this is important for our logic so we need to update it more frequently
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, m_motorPosition, m_motorCurrent, m_motorAcceleration);

    // all of these are for logging so we can use a lower frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connectedMotor,
        m_motorVelocity,
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
              m_motorAcceleration,
              m_motorCurrent,
              m_motorStatorCurrent,
              m_motorVoltage,
              m_motorTemperature));
    }
  }

  @Override
  public void updateInputs(ManipulatorRollerInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_connectedMotor,
          m_motorPosition,
          m_motorVelocity,
          m_motorAcceleration,
          m_motorCurrent,
          m_motorStatorCurrent,
          m_motorVoltage,
          m_motorTemperature);
    }

    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.positionDegrees = m_motorPosition.getValue().in(Degrees);
    inputs.positionControl = m_positionControl;
    inputs.desiredPositionDegrees = m_desiredPosition.in(Degrees);
    inputs.velocityRPS = m_motorVelocity.getValueAsDouble();
    inputs.accelerationRPSSq = m_motorAcceleration.getValueAsDouble();
    inputs.current = m_motorCurrent.getValueAsDouble();
    inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
    inputs.voltage = m_motorVoltage.getValueAsDouble();
    inputs.temperature = m_motorTemperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setControl(m_voltageOut.withOutput(voltage));
    m_positionControl = false;
    m_desiredPosition = Degrees.of(0.0);
  }

  @Override
  public void setDesiredPosition(Angle position) {
    m_motor.setControl(m_positionVoltage.withPosition(position));
    m_positionControl = true;
    m_desiredPosition = position;
  }

  @Override
  public Angle getPosition() {
    return m_motorPosition.getValue();
  }

  @Override
  public void setPositionPID(double kP, double kI, double kD, double kS) {
    m_motor
        .getConfigurator()
        .apply(
            m_config.Slot0.withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign),
            0.0);
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }

  @Override
  public boolean withinPositionTolerance() {
    return Math.abs(m_motorPosition.getValue().minus(m_desiredPosition).in(Degrees))
        < ManipulatorConstants.kRollerPositionTolerance;
  }

  @Override
  public double getCurrent() {
    return m_motorCurrent.getValueAsDouble();
  }

  @Override
  public double getAcceleration() {
    return m_motorAcceleration.getValueAsDouble();
  }
}
