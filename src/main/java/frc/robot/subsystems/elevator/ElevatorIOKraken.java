package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Ports;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOKraken implements ElevatorIO {
  private TalonFX m_leadingMotor;
  private TalonFX m_followingMotor;

  private final TalonFXConfiguration m_config;

  // private MotionMagicTorqueCurrentFOC m_magicMotion = new MotionMagicTorqueCurrentFOC(0);
  // private PositionTorqueCurrentFOC m_positionControl = new PositionTorqueCurrentFOC(0.0);
  private PositionVoltage m_positionControl = new PositionVoltage(0.0);
  // private MotionMagicVoltage m_magicMotion = new MotionMagicVoltage(0).withEnableFOC(true);

  // Status Signals
  private StatusSignal<ConnectedMotorValue> m_leadingConnectedMotor;
  private StatusSignal<ConnectedMotorValue> m_followingConnectedMotor;
  private StatusSignal<Angle> m_leadingPosition;
  private StatusSignal<Angle> m_followingPosition;
  private StatusSignal<AngularVelocity> m_leadingVelocity;
  private StatusSignal<AngularVelocity> m_followingVelocity;
  private StatusSignal<AngularAcceleration> m_leadingAcceleration;
  private StatusSignal<AngularAcceleration> m_followingAcceleration;
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

    var currentConfigs =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kElevatorDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kElevatorDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.kSensorToMechanismRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
            .withFeedback(feedbackConfig)
            .withCurrentLimits(currentConfigs)
            .withMotorOutput(motorOutput);

    m_leadingMotor.getConfigurator().apply(m_config);
    m_followingMotor.getConfigurator().apply(m_config);

    m_leadingMotor.getConfigurator().setPosition(0.0);
    m_followingMotor.getConfigurator().setPosition(0.0);
    m_followingMotor.setControl(new Follower(leadPort, true));

    m_leadingConnectedMotor = m_leadingMotor.getConnectedMotor();
    m_followingConnectedMotor = m_followingMotor.getConnectedMotor();
    m_leadingPosition = m_leadingMotor.getPosition();
    m_followingPosition = m_followingMotor.getPosition();
    m_leadingVelocity = m_leadingMotor.getVelocity();
    m_followingVelocity = m_followingMotor.getVelocity();
    m_leadingAcceleration = m_leadingMotor.getAcceleration();
    m_followingAcceleration = m_followingMotor.getAcceleration();
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
        m_leadingConnectedMotor,
        m_followingConnectedMotor,
        m_leadingVelocity,
        m_followingVelocity,
        m_leadingAcceleration,
        m_followingAcceleration,
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
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_leadingConnectedMotor,
          m_followingConnectedMotor,
          m_leadingPosition,
          m_leadingVelocity,
          m_leadingAcceleration,
          m_leadingVoltage,
          m_leadingSupplyCurrent,
          m_leadingStatorCurrent,
          m_leadingTemp,
          m_followingPosition,
          m_followingVelocity,
          m_followingAcceleration,
          m_followingVoltage,
          m_followingSupplyCurrent,
          m_followingStatorCurrent,
          m_followingTemp);
    }

    inputs.isLeadingMotorConnected =
        m_leadingConnectedMotor.getValue() != ConnectedMotorValue.Unknown;
    inputs.isFollowingMotorConnected =
        m_followingConnectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.atSetpoint = atSetpoint();
    inputs.desiredLocation = m_desiredHeight;
    inputs.leadingPosition = m_leadingPosition.getValueAsDouble();
    inputs.followingPosition = m_followingPosition.getValueAsDouble();
    inputs.leadingVelocity = m_leadingVelocity.getValueAsDouble();
    inputs.followingVelocity = m_followingVelocity.getValueAsDouble();
    inputs.leadingAcceleration = m_leadingAcceleration.getValueAsDouble();
    inputs.followingAcceleration = m_followingAcceleration.getValueAsDouble();
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
  public void setDesiredHeight(double inches) {
    // feedback
    m_desiredHeight = inches;
    // m_leadingMotor.setControl(m_magicMotion.withPosition(meters));
    m_leadingMotor.setControl(m_positionControl.withPosition(inches));
    // m_leadingMotor.setControl(new MotionMagicVoltage(meters).withSlot(0));
    // m_leadingMotor.setControl(new VoltageOut(3));
  }

  @Override
  public void setPIDFF(
      int slot, double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Logger.recordOutput("set ks ", kS);
    var slotConfigs =
        new SlotConfigs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withKG(kG);
    slotConfigs.SlotNumber = slot;
    m_leadingMotor.getConfigurator().apply(slotConfigs, 0.0);
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit);
    m_leadingMotor.getConfigurator().apply(m_config.CurrentLimits, 0.0);
    m_followingMotor.getConfigurator().apply(m_config.CurrentLimits, 0.0);
  }

  @Override
  public void setSlot(int slot) {
    m_positionControl.withSlot(slot);
  }

  @Override
  public void setMagic(double velocity, double acceleration, double jerk) {
    m_leadingMotor
        .getConfigurator()
        .apply(
            m_config.MotionMagic.withMotionMagicCruiseVelocity(velocity)
                .withMotionMagicAcceleration(acceleration)
                .withMotionMagicJerk(jerk),
            0.0);
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(m_leadingPosition.getValueAsDouble() - m_desiredHeight)
        <= ElevatorConstants.kHeightTolerance;
  }

  @Override
  public double getCurrHeight() {
    return m_leadingPosition.getValueAsDouble();
  }

  @Override
  public void zeroElevator() {
    m_leadingMotor.getConfigurator().setPosition(0.0, 0.0);
    m_followingMotor.getConfigurator().setPosition(0.0, 0.0);
  }
}
