package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOKraken implements ClimbIO {
  private TalonFX m_motor;

  private Slot0Configs m_Slot0Config = new Slot0Configs();
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);

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

    m_motor.getConfigurator().apply(config);
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    inputs.currPositionRad = Units.rotationsToRadians(m_motor.getPosition().getValueAsDouble());
    inputs.desiredPositionRad = m_desiredAngle.getRadians();
    inputs.atSetpoint =
        Math.abs(m_desiredAngle.getRadians() - inputs.currPositionRad)
            < ClimbConstants.kClimbTolerance.getRadians();
    inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.current = m_motor.getSupplyCurrent().getValueAsDouble();
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
        m_positionVoltage.withPosition(Units.radiansToRotations(angle.getRadians())));
  }

  @Override
  public void zeroEncoder() {
    m_motor.setPosition(0);
  }
}
