package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;

public class ElevatorIOKraken implements ElevatorIO {
  private TalonFX m_leadingMotor;
  private TalonFX m_followingMotor;
  private TalonFXConfiguration m_configs;
  private FeedbackConfigs m_feedbackConfigs;

  // TODO: re-enable when phoenix pro is purchased
  // private MotionMagicTorqueCurrentFOC m_magicMotion = new MotionMagicTorqueCurrentFOC(0);
  private MotionMagicVoltage m_magicMotion = new MotionMagicVoltage(0);

  // Status Signals
  private StatusSignal<Angle> m_leadingPosition;
  private StatusSignal<Angle> m_followingPosition;
  private StatusSignal<Voltage> m_leadingVoltage;
  private StatusSignal<Voltage> m_followingVoltage;
  private StatusSignal<Current> m_leadingSupplyCurrent;
  private StatusSignal<Current> m_followingSupplyCurrent;
  private StatusSignal<Current> m_leadingStatorCurrent;
  private StatusSignal<Current> m_followingStatorCurrent;
  private StatusSignal<Temperature> m_leadingTemp;
  private StatusSignal<Temperature> m_followingTemp;

  private double m_desiredHeight;

  public ElevatorIOKraken(int leadPort, int followerPort) {
    m_leadingMotor = new TalonFX(leadPort, Ports.kMainCanivoreName);
    m_followingMotor = new TalonFX(followerPort, Ports.kMainCanivoreName);

    m_configs = new TalonFXConfiguration();

    var currentConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kElevatorDefaultStatorLimit)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kElevatorDefaultSupplyLimit);

    m_feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.kSensorToMechanismRatio);

    m_configs = m_configs.withFeedback(m_feedbackConfigs).withCurrentLimits(currentConfigs);

    m_leadingMotor.getConfigurator().apply(m_configs);
    m_followingMotor.getConfigurator().apply(m_configs);
    m_followingMotor.setControl(new Follower(leadPort, true));

    m_leadingPosition = m_leadingMotor.getPosition();
    m_followingPosition = m_followingMotor.getPosition();
    m_leadingVoltage = m_leadingMotor.getMotorVoltage();
    m_followingVoltage = m_followingMotor.getMotorVoltage();
    m_leadingSupplyCurrent = m_leadingMotor.getSupplyCurrent();
    m_followingSupplyCurrent = m_followingMotor.getSupplyCurrent();
    m_leadingStatorCurrent = m_leadingMotor.getStatorCurrent();
    m_followingStatorCurrent = m_followingMotor.getStatorCurrent();
    m_leadingTemp = m_leadingMotor.getDeviceTemp();
    m_followingTemp = m_followingMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(100, m_leadingPosition, m_followingPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        75,
        m_leadingVoltage,
        m_followingVoltage,
        m_leadingSupplyCurrent,
        m_followingSupplyCurrent,
        m_leadingStatorCurrent,
        m_followingStatorCurrent,
        m_leadingTemp,
        m_followingTemp);

    ParentDevice.optimizeBusUtilizationForAll(m_leadingMotor, m_followingMotor);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.isLeadingMotorConnected =
        BaseStatusSignal.refreshAll(
                m_leadingPosition,
                m_leadingVoltage,
                m_leadingSupplyCurrent,
                m_leadingStatorCurrent,
                m_leadingTemp)
            .isOK();
    inputs.isFollowingMotorConnected =
        BaseStatusSignal.refreshAll(
                m_followingPosition,
                m_followingVoltage,
                m_followingSupplyCurrent,
                m_followingStatorCurrent,
                m_followingTemp)
            .isOK();
    inputs.atSetpoint = atSetpoint();
    inputs.desiredLocation = m_desiredHeight;
    inputs.leadingPosition = m_leadingPosition.getValueAsDouble();
    inputs.followingPosition = m_followingPosition.getValueAsDouble();
    inputs.leadingVelocity = m_leadingMotor.getVelocity().getValueAsDouble();
    inputs.followingVelocity = m_followingMotor.getVelocity().getValueAsDouble();
    inputs.leadingVoltage = m_leadingVoltage.getValueAsDouble();
    inputs.followingVoltage = m_followingVoltage.getValueAsDouble();
    inputs.leadingSupplyCurrent = m_leadingSupplyCurrent.getValueAsDouble();
    inputs.followingSupplyCurrent = m_followingSupplyCurrent.getValueAsDouble();
    inputs.leadingStatorCurrent = m_leadingStatorCurrent.getValueAsDouble();
    inputs.followingStatorCurrent = m_followingStatorCurrent.getValueAsDouble();
    inputs.leadingTemp = m_leadingTemp.getValueAsDouble();
    inputs.followingTemp = m_followingTemp.getValueAsDouble();
  }

  @Override
  public void setDesiredHeight(double meters) {
    // feedback
    m_desiredHeight = meters;
    m_leadingMotor.setControl(m_magicMotion.withPosition(meters).withSlot(0));
    // m_leadingMotor.setControl(new MotionMagicVoltage(meters).withSlot(0));
    // m_leadingMotor.setControl(new VoltageOut(3));
  }

  @Override
  public void setPIDFF(
      int slot, double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    if (slot == 0) {
      m_leadingMotor
          .getConfigurator()
          .apply(
              m_configs.Slot0.withKP(kP)
                  .withKI(kI)
                  .withKD(kD)
                  .withKS(kS)
                  .withKV(kV)
                  .withKA(kA)
                  .withKG(kG),
              0.0);
    } else if (slot == 1) {
      m_leadingMotor
          .getConfigurator()
          .apply(
              m_configs.Slot1.withKP(kP)
                  .withKI(kI)
                  .withKD(kD)
                  .withKS(kS)
                  .withKV(kV)
                  .withKA(kA)
                  .withKG(kG),
              0.0);
    } else {
      m_leadingMotor
          .getConfigurator()
          .apply(
              m_configs.Slot2.withKP(kP)
                  .withKI(kI)
                  .withKD(kD)
                  .withKS(kS)
                  .withKV(kV)
                  .withKA(kA)
                  .withKG(kG),
              0.0);
    }
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    var newConfigs = m_configs.CurrentLimits;
    newConfigs.StatorCurrentLimit = supplyLimit;
    newConfigs.SupplyCurrentLimit = supplyLimit;
    m_leadingMotor.getConfigurator().apply(newConfigs, 0.0);
  }

  @Override
  public void setSlot(int slot) {
    if (slot == 0) {
      m_leadingMotor.getConfigurator().apply(m_configs.Slot0, 0.0);
      m_magicMotion.withSlot(0);
    }
    if (slot == 1) {
      m_leadingMotor.getConfigurator().apply(m_configs.Slot1, 0.0);
      m_magicMotion.withSlot(1);
    }
    if (slot == 2) {
      m_leadingMotor.getConfigurator().apply(m_configs.Slot2, 0.0);
      m_magicMotion.withSlot(2);
    }
  }

  @Override
  public void setMagic(double velocity, double acceleration, double jerk) {
    var magic =
        m_configs.MotionMagic.withMotionMagicCruiseVelocity(velocity)
            .withMotionMagicAcceleration(acceleration)
            .withMotionMagicJerk(jerk);
    m_leadingMotor.getConfigurator().apply(magic, 0.0);
  }

  @Override
  public boolean atSetpoint() {
    return (m_leadingPosition.getValueAsDouble() - m_desiredHeight)
        <= ElevatorConstants.kHeightTolerance;
  }

  @Override
  public double getCurrHeight() {
    return m_leadingPosition.getValueAsDouble();
  }
}
