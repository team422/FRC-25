package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class DriveIOCIM implements DriveIO {
  private SparkMax m_LF;
  private SparkMax m_LR;
  private SparkMax m_RF;
  private SparkMax m_RR;

  public DriveIOCIM() {
    m_LF = new SparkMax(Constants.Ports.kLFPort, MotorType.kBrushed);
    m_LR = new SparkMax(Constants.Ports.kLRPort, MotorType.kBrushed);
    m_RF = new SparkMax(Constants.Ports.KRFPort, MotorType.kBrushed);
    m_RR = new SparkMax(Constants.Ports.kRRPort, MotorType.kBrushed);

    this.configureMotors();
  }

  public void updateInputs(DriveInputs inputs) {
    inputs.voltage = new double[] {m_LF.getBusVoltage() * m_LF.getAppliedOutput()};
    inputs.current = new double[] {m_LF.getOutputCurrent()};
  }

  public void setLeft(double value) {
    m_LF.set(value);
    m_LR.set(value);
  }

  public void setRight(double value) {
    m_RF.set(value);
    m_RR.set(value);
  }

  private void configureMotors() {
    SparkMaxConfig m_frontConfig = new SparkMaxConfig();
    m_frontConfig.idleMode(IdleMode.kCoast);
    m_LF.configure(
        m_frontConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); // front left
    m_RF.configure(
        m_frontConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); // front right

    SparkMaxConfig m_LRConfig = new SparkMaxConfig();
    m_LRConfig.idleMode(IdleMode.kCoast);
    m_LRConfig.follow(m_LF, true);
    m_LR.configure(m_LRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig m_RRConfig = new SparkMaxConfig();
    m_RRConfig.idleMode(IdleMode.kCoast);
    m_RRConfig.follow(m_RF, true);
    m_RR.configure(m_RRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
