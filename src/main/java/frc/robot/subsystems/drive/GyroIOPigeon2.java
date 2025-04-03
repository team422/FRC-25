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

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_pigeon = new Pigeon2(Ports.kPigeon, Ports.kDriveCanivoreName);
  private final StatusSignal<Angle> m_yaw = m_pigeon.getYaw();
  private final StatusSignal<Angle> m_pitch = m_pigeon.getPitch();
  private final StatusSignal<Angle> m_roll = m_pigeon.getRoll();
  private final Queue<Double> m_yawPositionQueue;
  private final Queue<Double> m_pitchPositionQueue;
  private final Queue<Double> m_rollPositionQueue;
  private final Queue<Double> m_zAccelerationQueue;
  private final Queue<Double> m_timestampQueue;
  private final StatusSignal<AngularVelocity> m_yawVelocity = m_pigeon.getAngularVelocityZWorld();
  private final StatusSignal<AngularVelocity> m_pitchVelocity = m_pigeon.getAngularVelocityXWorld();
  private final StatusSignal<AngularVelocity> m_rollVelocity = m_pigeon.getAngularVelocityYWorld();
  private final StatusSignal<LinearAcceleration> m_xAcceleration = m_pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> m_yAcceleration = m_pigeon.getAccelerationY();
  private final StatusSignal<LinearAcceleration> m_zAcceleration = m_pigeon.getAccelerationZ();
  private final StatusSignal<Voltage> m_pigeonSupplyVoltage = m_pigeon.getSupplyVoltage();

  public GyroIOPigeon2() {
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
    m_pigeon.getConfigurator().setYaw(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.kOdometryFrequency,
        m_yaw,
        m_pitch,
        m_roll,
        m_yawVelocity,
        m_pitchVelocity,
        m_rollVelocity,
        m_xAcceleration,
        m_yAcceleration,
        m_zAcceleration);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, m_pigeonSupplyVoltage);

    // m_pigeon.optimizeBusUtilization();

    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getYaw());
    m_pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getPitch());
    m_rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getRoll());
    m_zAccelerationQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(m_pigeon.getAccelerationZ()::getValueAsDouble);

    if (Constants.kUseBaseRefreshManager) {
      CtreBaseRefreshManager.addSignals(
          List.of(
              m_yaw,
              m_pitch,
              m_roll,
              m_yawVelocity,
              m_pitchVelocity,
              m_rollVelocity,
              m_pigeonSupplyVoltage));
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_yaw,
          m_pitch,
          m_roll,
          m_yawVelocity,
          m_pitchVelocity,
          m_rollVelocity,
          m_xAcceleration,
          m_yAcceleration,
          m_zAcceleration,
          m_pigeonSupplyVoltage);
    }

    // this is a hack because the pigeon doesn't have a connected motor we can check (obviously)
    // so basically if we're in sim, the supply voltage is EXACTLY zero
    // if we connected it'll be something else
    inputs.connected = m_pigeonSupplyVoltage.getValueAsDouble() != 0.0;

    if (RobotBase.isSimulation()) {
      inputs.connected = false;
    }

    inputs.yawPosition = Rotation2d.fromDegrees(m_yaw.getValueAsDouble());
    inputs.pitchPosition = Rotation2d.fromDegrees(m_pitch.getValueAsDouble());
    inputs.rollPosition = Rotation2d.fromDegrees(m_roll.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_yawVelocity.getValueAsDouble());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(m_pitchVelocity.getValueAsDouble());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(m_rollVelocity.getValueAsDouble());
    inputs.xAcceleration = m_xAcceleration.getValue().in(MetersPerSecondPerSecond);
    inputs.yAcceleration = m_yAcceleration.getValue().in(MetersPerSecondPerSecond);
    inputs.zAcceleration = m_zAcceleration.getValue().in(MetersPerSecondPerSecond);

    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryPitchPositions =
        m_pitchPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryRollPositions =
        m_rollPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryZAccelerations =
        m_zAccelerationQueue.stream().mapToDouble((Double value) -> value).toArray();

    m_timestampQueue.clear();
    m_yawPositionQueue.clear();
    m_pitchPositionQueue.clear();
    m_rollPositionQueue.clear();
    m_zAccelerationQueue.clear();
  }
}
