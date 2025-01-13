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

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;

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

  private final StatusSignal<Angle> m_drivePosition;
  private final Queue<Double> m_drivePositionQueue;
  private final StatusSignal<AngularVelocity> m_driveVelocity;
  private final StatusSignal<Voltage> m_driveAppliedVolts;
  private final StatusSignal<Current> m_driveCurrent;

  private final StatusSignal<Angle> m_turnAbsolutePosition;
  private final StatusSignal<Angle> m_turnPosition;
  private final Queue<Double> m_turnPositionQueue;
  private final StatusSignal<AngularVelocity> m_turnVelocity;
  private final StatusSignal<Voltage> m_turnAppliedVolts;
  private final StatusSignal<Current> m_turnCurrent;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        m_driveTalon = new TalonFX(Ports.kFrontLeftDrive, Ports.kCanivoreName);
        m_turnTalon = new TalonFX(Ports.kFrontLeftTurn, Ports.kCanivoreName);
        m_cancoder = new CANcoder(Ports.kFrontLeftCancoder, Ports.kCanivoreName);
        absoluteEncoderOffset = new Rotation2d(1.617); // MUST BE CALIBRATED
        break;
      case 1:
        m_driveTalon = new TalonFX(Ports.kFrontRightDrive, Ports.kCanivoreName);
        m_turnTalon = new TalonFX(Ports.kFrontRightTurn, Ports.kCanivoreName);
        m_cancoder = new CANcoder(Ports.kFrontRightCancoder, Ports.kCanivoreName);
        absoluteEncoderOffset = new Rotation2d(2.750); // MUST BE CALIBRATED
        break;
      case 2:
        m_driveTalon = new TalonFX(Ports.kBackLeftDrive, Ports.kCanivoreName);
        m_turnTalon = new TalonFX(Ports.kBackLeftTurn, Ports.kCanivoreName);
        m_cancoder = new CANcoder(Ports.kBackLeftCancoder, Ports.kCanivoreName);
        absoluteEncoderOffset = new Rotation2d(-0.948); // MUST BE CALIBRATED
        break;
      case 3:
        m_driveTalon = new TalonFX(Ports.kBackRightDrive, Ports.kCanivoreName);
        m_turnTalon = new TalonFX(Ports.kBackRightTurn, Ports.kCanivoreName);
        m_cancoder = new CANcoder(Ports.kBackRightCancoder, Ports.kCanivoreName);
        absoluteEncoderOffset = new Rotation2d(2.049); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    m_cancoder.getConfigurator().apply(new CANcoderConfiguration());

    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    m_drivePosition = m_driveTalon.getPosition();
    m_drivePositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(m_driveTalon.getPosition());
    m_driveVelocity = m_driveTalon.getVelocity();
    m_driveAppliedVolts = m_driveTalon.getMotorVoltage();
    m_driveCurrent = m_driveTalon.getSupplyCurrent();

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
    BaseStatusSignal.refreshAll(
        m_drivePosition,
        m_driveVelocity,
        m_driveAppliedVolts,
        m_driveCurrent,
        m_turnAbsolutePosition,
        m_turnPosition,
        m_turnVelocity,
        m_turnAppliedVolts,
        m_turnCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(m_drivePosition.getValueAsDouble())
            / DriveConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(m_driveVelocity.getValueAsDouble())
            / DriveConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = m_driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {m_driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(m_turnPosition.getValueAsDouble() / DriveConstants.kTurnGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(m_turnVelocity.getValueAsDouble()) / DriveConstants.kTurnGearRatio;
    inputs.turnAppliedVolts = m_turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {m_turnCurrent.getValueAsDouble()};

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
  public void setDriveVoltage(double volts) {
    m_driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_turnTalon.getConfigurator().apply(config);
  }
}
