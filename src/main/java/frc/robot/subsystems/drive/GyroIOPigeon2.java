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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
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
  private final Queue<Double> m_timestampQueue;
  private final StatusSignal<AngularVelocity> m_yawVelocity = m_pigeon.getAngularVelocityZWorld();
  private final StatusSignal<AngularVelocity> m_pitchVelocity = m_pigeon.getAngularVelocityXWorld();
  private final StatusSignal<AngularVelocity> m_rollVelocity = m_pigeon.getAngularVelocityYWorld();

  public GyroIOPigeon2() {
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
    m_pigeon.getConfigurator().setYaw(0.0);

    m_yaw.setUpdateFrequency(DriveConstants.kOdometryFrequency);
    m_pitch.setUpdateFrequency(DriveConstants.kOdometryFrequency);
    m_roll.setUpdateFrequency(DriveConstants.kOdometryFrequency);

    m_yawVelocity.setUpdateFrequency(100.0);
    m_pitchVelocity.setUpdateFrequency(100.0);
    m_rollVelocity.setUpdateFrequency(100.0);

    m_pigeon.optimizeBusUtilization();

    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getYaw());
    m_pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getPitch());
    m_rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getRoll());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                m_yaw, m_pitch, m_roll, m_yawVelocity, m_pitchVelocity, m_rollVelocity)
            .equals(StatusCode.OK);

    if (RobotBase.isSimulation()) {
      inputs.connected = false;
    }

    inputs.yawPosition = Rotation2d.fromDegrees(m_yaw.getValueAsDouble());
    inputs.pitchPosition = Rotation2d.fromDegrees(m_pitch.getValueAsDouble());
    inputs.rollPosition = Rotation2d.fromDegrees(m_roll.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_yawVelocity.getValueAsDouble());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(m_pitchVelocity.getValueAsDouble());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(m_rollVelocity.getValueAsDouble());

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

    m_timestampQueue.clear();
    m_yawPositionQueue.clear();
    m_pitchPositionQueue.clear();
    m_rollPositionQueue.clear();
  }
}
