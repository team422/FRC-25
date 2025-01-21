package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOKraken implements ElevatorIO {
  private TalonFX m_leadingMotor;
  private TalonFX m_followingMotor;
  private Slot0Configs m_slot0Configs = new Slot0Configs();
  private TalonFXConfiguration m_configs;
  private MotionMagicVoltage m_voltageOut = new MotionMagicVoltage(0);
  private double desiredHeight;

  public ElevatorIOKraken(int leadPort, int followerPort) {
    m_leadingMotor = new TalonFX(leadPort);
    m_followingMotor = new TalonFX(followerPort);

    m_configs = new TalonFXConfiguration();
    m_configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_configs.CurrentLimits.SupplyCurrentLimit = 30.0;
    m_configs.CurrentLimits.StatorCurrentLimitEnable = true;
    m_configs.CurrentLimits.StatorCurrentLimit = 30.0;

    m_leadingMotor.getConfigurator().apply(m_configs);
    m_followingMotor.getConfigurator().apply(m_configs);
    m_followingMotor.setControl(
        new Follower(leadPort, true)); // can change idk if its meant to be opposite
  }

  @Override
  public void updateInputs(ElevatorInputsAutoLogged inputs) {
    // TODO Auto-generated method stub
    inputs.atSetpoint =
        (m_followingMotor.getPosition().getValueAsDouble() - desiredHeight)
            <= ElevatorConstants.kHeightTolerance;
    inputs.currLocation = m_leadingMotor.getPosition().getValueAsDouble();
    inputs.desiredLocation = desiredHeight;
    inputs.voltage = m_leadingMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setDesiredHeight(double centimeters) {
    // TODO Auto-generated method stub
    desiredHeight = centimeters;
    m_leadingMotor.setControl(m_voltageOut.withPosition(centimeters));
  }

  @Override
  public void setPIDFF(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    // TODO Auto-generated method stub
    m_slot0Configs.kP = kP;
    m_slot0Configs.kI = kI;
    m_slot0Configs.kD = kD;
    m_slot0Configs.kS = kS;
    m_slot0Configs.kV = kV;
    m_slot0Configs.kG = kG;
    m_slot0Configs.kA = kA;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.SensorToMechanismRatio = ElevatorConstants.kRadius;

    var motionMagicConfigs = m_configs.MotionMagic;
    motionMagicConfigs.MotionMagicExpo_kA = kA;
    motionMagicConfigs.MotionMagicExpo_kV = kV;

    m_configs = m_configs.withFeedback(feedbackConfigs).withMotionMagic(motionMagicConfigs);

    m_leadingMotor.getConfigurator().apply(m_configs);
  }
}
