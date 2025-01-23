package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOKraken implements ClimbIO {
  private TalonFX m_motor;

  private Slot0Configs m_Slot0Config = new Slot0Configs();
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);

  // Status Signals
  private StatusSignal<Angle> m_motorPosition;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Temperature> m_motorTemperature;

  public ClimbIOKraken(int port) {
    m_motor = new TalonFX(port);

    var currentLimitConfigs =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(120);

    var feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(ClimbConstants.kClimbReduction);

    var config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withFeedback(feedbackConfigs);

    // init StatusSignals (100hz)
    m_motorPosition = m_motor.getPosition();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, m_motorPosition, m_motorVoltage, m_motorCurrent, m_motorStatorCurrent);

    // init 75hz signals (temp)
    m_motorTemperature = m_motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(75, m_motorTemperature);

    // optimize signal use

    m_motor.optimizeBusUtilization(0, 1.0);

    m_motor.getConfigurator().apply(config);
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    inputs.motorIsConnected =
        BaseStatusSignal.refreshAll(
                m_motorPosition,
                m_motorVoltage,
                m_motorCurrent,
                m_motorStatorCurrent,
                m_motorTemperature)
            .isOK();

    inputs.currPositionDegrees = getCurrPosition().getDegrees();
    inputs.desiredPositionDegrees = m_desiredAngle.getDegrees();
    inputs.atSetpoint = atSetpoint();
    inputs.voltage = m_motorVoltage.getValueAsDouble();
    inputs.current = m_motorCurrent.getValueAsDouble();
    inputs.statorCurrent = m_motorStatorCurrent.getValueAsDouble();
    inputs.temperature = m_motorTemperature.getValueAsDouble();
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    m_Slot0Config.kP = kP;
    m_Slot0Config.kI = kI;
    m_Slot0Config.kD = kD;

    m_motor.getConfigurator().apply(m_Slot0Config);
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    m_desiredAngle = angle;
    m_motor.setControl(
        m_positionVoltage.withPosition(angle.getDegrees()));
  }

  @Override
  public void zeroEncoder() {
    m_motor.setPosition(0);
  }

  public Rotation2d getCurrPosition() {
    return Rotation2d.fromRotations(m_motorPosition.getValueAsDouble());
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredAngle.getDegrees() - getCurrPosition().getDegrees())
        < ClimbConstants.kClimbTolerance;
  }
}
