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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final PIDController m_driveFeedback;
  private final PIDController m_turnFeedback;
  private Rotation2d m_angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double m_speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d m_turnRelativeOffset = null; // Relative + Offset = Absolute
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.m_io = io;
    this.m_index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    if (RobotBase.isReal()) {
      m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
      m_driveFeedback = new PIDController(0.05, 0.0, 0.0);
      m_turnFeedback = new PIDController(7.0, 0.0, 0.0);
    } else {
      m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
      m_driveFeedback = new PIDController(0.1, 0.0, 0.0);
      m_turnFeedback = new PIDController(10.0, 0.0, 0.0);
    }

    m_turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    m_io.updateInputs(m_inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(m_index), m_inputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (m_turnRelativeOffset == null && m_inputs.turnAbsolutePosition.getRadians() != 0.0) {
      m_turnRelativeOffset = m_inputs.turnAbsolutePosition.minus(m_inputs.turnPosition);
    }

    // Run closed loop turn control
    if (m_angleSetpoint != null) {
      m_io.setTurnVoltage(
          m_turnFeedback.calculate(getAngle().getRadians(), m_angleSetpoint.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (m_speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90 degrees, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = m_speedSetpoint * Math.cos(m_turnFeedback.getError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / DriveConstants.kWheelRadius;
        m_io.setDriveVoltage(
            m_driveFeedforward.calculate(velocityRadPerSec)
                + m_driveFeedback.calculate(m_inputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }

    // Calculate positions for odometry
    int sampleCount = m_inputs.odometryTimestamps.length; // All signals are sampled together
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * DriveConstants.kWheelRadius;
      Rotation2d angle =
          m_inputs.odometryTurnPositions[i].plus(
              m_turnRelativeOffset != null ? m_turnRelativeOffset : new Rotation2d());
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null

    var optimizedState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    optimizedState.optimize(getAngle());

    // Update setpoints, controllers run in "periodic"
    m_angleSetpoint = optimizedState.angle;
    m_speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    m_angleSetpoint = new Rotation2d();

    // Open loop drive control
    m_io.setDriveVoltage(volts);
    m_speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    m_io.setTurnVoltage(0.0);
    m_io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    m_angleSetpoint = null;
    m_speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
    m_io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (m_turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return m_inputs.turnPosition.plus(m_turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * DriveConstants.kWheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * DriveConstants.kWheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return m_odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }

  public void setCurrentLimits(double supplyLimit) {
    m_io.setCurrentLimits(supplyLimit);
  }
}
