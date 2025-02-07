// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX m_driveTalon;
  private final TalonFX m_turnTalon;
  private final CANcoder m_cancoder;

  private final Queue<Double> m_timestampQueue;

  private final StatusSignal<ConnectedMotorValue> m_driveConnectedMotor;
  private final StatusSignal<Angle> m_drivePosition;
  private final Queue<Double> m_drivePositionQueue;
  private final StatusSignal<AngularVelocity> m_driveVelocity;
  private final StatusSignal<Voltage> m_driveAppliedVolts;
  private final StatusSignal<Current> m_driveCurrent;

  private final StatusSignal<ConnectedMotorValue> m_turnConnectedMotor;
  private final StatusSignal<Angle> m_turnAbsolutePosition;
  private final StatusSignal<Angle> m_turnPosition;
  private final Queue<Double> m_turnPositionQueue;
  private final StatusSignal<AngularVelocity> m_turnVelocity;
  private final StatusSignal<Voltage> m_turnAppliedVolts;
  private final StatusSignal<Current> m_turnCurrent;

  private final boolean m_isTurnMotorInverted = true;
  private final boolean m_isCancoderInverted = false;
  private final Rotation2d m_absoluteEncoderOffset;

  private final VelocityTorqueCurrentFOC m_driveControl = new VelocityTorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC m_turnControl = new PositionTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC m_rawControl = new TorqueCurrentFOC(0.0);

  private final TalonFXConfiguration m_driveConfig;
  private final TalonFXConfiguration m_turnConfig;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        m_driveTalon = new TalonFX(Ports.kFrontLeftDrive, Ports.kDriveCanivoreName);
        m_turnTalon = new TalonFX(Ports.kFrontLeftTurn, Ports.kDriveCanivoreName);
        m_cancoder = new CANcoder(Ports.kFrontLeftCancoder, Ports.kDriveCanivoreName);
        m_absoluteEncoderOffset = new Rotation2d(); // we set this in Phoenix Tuner so no need here
        break;
      case 1:
        m_driveTalon = new TalonFX(Ports.kFrontRightDrive, Ports.kDriveCanivoreName);
        m_turnTalon = new TalonFX(Ports.kFrontRightTurn, Ports.kDriveCanivoreName);
        m_cancoder = new CANcoder(Ports.kFrontRightCancoder, Ports.kDriveCanivoreName);
        m_absoluteEncoderOffset = new Rotation2d(); // we set this in Phoenix Tuner so no need here
        break;
      case 2:
        m_driveTalon = new TalonFX(Ports.kBackLeftDrive, Ports.kDriveCanivoreName);
        m_turnTalon = new TalonFX(Ports.kBackLeftTurn, Ports.kDriveCanivoreName);
        m_cancoder = new CANcoder(Ports.kBackLeftCancoder, Ports.kDriveCanivoreName);
        m_absoluteEncoderOffset = new Rotation2d(); // we set this in Phoenix Tuner so no need here
        break;
      case 3:
        m_driveTalon = new TalonFX(Ports.kBackRightDrive, Ports.kDriveCanivoreName);
        m_turnTalon = new TalonFX(Ports.kBackRightTurn, Ports.kDriveCanivoreName);
        m_cancoder = new CANcoder(Ports.kBackRightCancoder, Ports.kDriveCanivoreName);
        m_absoluteEncoderOffset = new Rotation2d(); // we set this in Phoenix Tuner so no need here
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveCurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kDriveDefaultSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kDriveDefaultStatorCurrentLimit);

    var driveSlot0Configs = new Slot0Configs().withKP(0).withKI(0).withKD(0);

    var driveFeedback =
        new FeedbackConfigs().withSensorToMechanismRatio(DriveConstants.kDriveGearRatio);

    var driveTorque =
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(CurrentLimitConstants.kDriveDefaultSupplyCurrentLimit)
            .withPeakReverseTorqueCurrent(-CurrentLimitConstants.kDriveDefaultSupplyCurrentLimit);

    var driveClosedLoopRamp = new ClosedLoopRampsConfigs().withTorqueClosedLoopRampPeriod(0.02);

    m_driveConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(driveCurrentLimits)
            .withSlot0(driveSlot0Configs)
            .withFeedback(driveFeedback)
            .withTorqueCurrent(driveTorque)
            .withClosedLoopRamps(driveClosedLoopRamp);
    m_driveTalon.getConfigurator().apply(m_driveConfig);
    m_driveTalon.setPosition(0.0);
    setDriveBrakeMode(true);

    var turnCurrentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kTurnDefaultSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kTurnDefaultStatorCurrentLimit);

    var turnSlot0Configs = new Slot0Configs().withKP(0).withKI(0).withKD(0);

    var turnFeedback =
        new FeedbackConfigs()
            .withSensorToMechanismRatio(DriveConstants.kTurnGearRatio)
            .withFusedCANcoder(m_cancoder);

    var turnTorque =
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(CurrentLimitConstants.kTurnDefaultSupplyCurrentLimit)
            .withPeakReverseTorqueCurrent(-CurrentLimitConstants.kTurnDefaultSupplyCurrentLimit);

    var turnClosedLoopGeneral = new ClosedLoopGeneralConfigs();
    // there's no method chaining option for ContinuousWrap for some reason
    turnClosedLoopGeneral.ContinuousWrap = true;

    m_turnConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(turnCurrentLimits)
            .withSlot0(turnSlot0Configs)
            .withFeedback(turnFeedback)
            .withTorqueCurrent(turnTorque)
            .withClosedLoopGeneral(turnClosedLoopGeneral);
    m_turnTalon.getConfigurator().apply(m_turnConfig);
    setTurnBrakeMode(true);

    // we want to keep the existing offset so we can set them in phoenix tuner rather than code
    var currMagnet = new MagnetSensorConfigs();
    m_cancoder.getConfigurator().refresh(currMagnet);

    m_cancoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    currMagnet.withSensorDirection(
                        m_isCancoderInverted
                            ? SensorDirectionValue.Clockwise_Positive
                            : SensorDirectionValue.CounterClockwise_Positive)));

    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    m_driveConnectedMotor = m_driveTalon.getConnectedMotor();
    m_drivePosition = m_driveTalon.getPosition();
    m_drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_driveTalon.getPosition());
    m_driveVelocity = m_driveTalon.getVelocity();
    m_driveAppliedVolts = m_driveTalon.getMotorVoltage();
    m_driveCurrent = m_driveTalon.getSupplyCurrent();

    m_turnConnectedMotor = m_turnTalon.getConnectedMotor();
    m_turnAbsolutePosition = m_cancoder.getAbsolutePosition();
    m_turnPosition = m_turnTalon.getPosition();
    m_turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(m_turnTalon.getPosition());
    m_turnVelocity = m_turnTalon.getVelocity();
    m_turnAppliedVolts = m_turnTalon.getMotorVoltage();
    m_turnCurrent = m_turnTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.kOdometryFrequency, m_drivePosition, m_turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_driveConnectedMotor,
        m_turnConnectedMotor,
        m_driveVelocity,
        m_driveAppliedVolts,
        m_driveCurrent,
        m_turnAbsolutePosition,
        m_turnVelocity,
        m_turnAppliedVolts,
        m_turnCurrent);
    m_driveTalon.optimizeBusUtilization();
    m_turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_driveConnectedMotor,
          m_drivePosition,
          m_driveVelocity,
          m_driveAppliedVolts,
          m_driveCurrent,
          m_turnConnectedMotor,
          m_turnAbsolutePosition,
          m_turnPosition,
          m_turnVelocity,
          m_turnAppliedVolts,
          m_turnCurrent);
    }

    inputs.drivePositionRad =
        Units.rotationsToRadians(m_drivePosition.getValueAsDouble())
            / DriveConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(m_driveVelocity.getValueAsDouble())
            / DriveConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = m_driveCurrent.getValueAsDouble();
    inputs.driveMotorIsConnected = m_driveConnectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble())
            .minus(m_absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(m_turnPosition.getValueAsDouble() / DriveConstants.kTurnGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(m_turnVelocity.getValueAsDouble()) / DriveConstants.kTurnGearRatio;
    inputs.turnAppliedVolts = m_turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = m_turnCurrent.getValueAsDouble();
    inputs.turnMotorIsConnected = m_turnConnectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value) / DriveConstants.kDriveGearRatio)
            .toArray();
    inputs.odometryTurnPositions =
        m_turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.kTurnGearRatio))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    m_driveTalon.setControl(
        m_driveControl
            .withVelocity(RadiansPerSecond.of(velocityRadPerSec))
            .withFeedForward(feedforward));
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    m_turnTalon.setControl(m_turnControl.withPosition(position.getMeasure()));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_driveTalon.getConfigurator().apply(config, 0.0);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        m_isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_turnTalon.getConfigurator().apply(config, 0.0);
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_driveTalon
        .getConfigurator()
        .apply(m_driveConfig.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
    m_driveTalon
        .getConfigurator()
        .apply(
            m_driveConfig
                .TorqueCurrent
                .withPeakForwardTorqueCurrent(supplyLimit)
                .withPeakReverseTorqueCurrent(-supplyLimit),
            0.0);
  }

  @Override
  public void setDriveRawOutput(double output) {
    m_driveTalon.setControl(m_rawControl.withOutput(output));
  }

  @Override
  public void setTurnRawOutput(double output) {
    m_turnTalon.setControl(m_rawControl.withOutput(output));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    m_driveTalon
        .getConfigurator()
        .apply(m_driveConfig.Slot0.withKP(kP).withKI(kI).withKD(kD).withKS(0), 0.0);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    m_turnTalon.getConfigurator().apply(m_turnConfig.Slot0.withKP(kP).withKI(kI).withKD(kD), 0.0);
  }
}
