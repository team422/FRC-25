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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private boolean m_turnRelativeReset = false; // whether we reset the turn motor already
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};

  private Alert m_driveDisconnectedAlert;
  private Alert m_turnDisconnectedAlert;
  private Alert m_canCoderDisconnectedAlert;

  public Module(ModuleIO io, int index) {
    this.m_io = io;
    this.m_index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    if (RobotBase.isReal()) {
      m_driveFeedforward = new SimpleMotorFeedforward(0.229, 0.138, 0.138);
      m_io.setDrivePID(1.0, 0.0, 0.0);
      m_io.setTurnPID(300.0, 0.0, 0.0);
    } else {
      m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13, 0.13);
      m_io.setDrivePID(0.1, 0.0, 0.0);
      m_io.setTurnPID(10.0, 0.0, 0.0);
    }

    setBrakeMode(true);

    m_driveDisconnectedAlert =
        new Alert(String.format("Drive %d Disconnect", m_index), AlertType.kError);
    m_turnDisconnectedAlert =
        new Alert(String.format("Turn %d Disconnect", m_index), AlertType.kError);
    m_canCoderDisconnectedAlert =
        new Alert(String.format("Cancoder %d Disconnect", m_index), AlertType.kError);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(m_index), m_inputs);
  }

  public void periodic() {
    if (!m_turnRelativeReset && m_inputs.turnEncoderIsConnected) {
      m_io.resetTurnMotor(m_inputs.turnAbsolutePosition.getMeasure());
      m_turnRelativeReset = true;
    }

    // Calculate positions for odometry
    int sampleCount = m_inputs.odometryTimestamps.length; // All signals are sampled together
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * DriveConstants.kWheelRadius;
      Rotation2d angle = m_inputs.odometryTurnPositions[i];
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    if (Constants.kUseAlerts && !m_inputs.driveMotorIsConnected) {
      m_driveDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    if (Constants.kUseAlerts && !m_inputs.turnMotorIsConnected) {
      m_turnDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }

    if (Constants.kUseAlerts && !m_inputs.turnEncoderIsConnected) {
      m_canCoderDisconnectedAlert.set(true);
      RobotState.getInstance().triggerAlert();
    }
  }

  /** Runs the module with the specified setpoint state. */
  public void runSetpoint(SwerveModuleState state) {
    state.optimize(getAngle());

    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.kWheelRadius;

    m_io.setDriveVelocity(speedRadPerSec, m_driveFeedforward.calculate(speedRadPerSec));
    m_io.setTurnPosition(state.angle);
  }

  /** */
  public void runSetpoint(SwerveModuleState state, LinearAcceleration accel) {
    state.optimize(getAngle());

    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.kWheelRadius;
    double accelRadPerSec =
        Math.signum(speedRadPerSec)
            * accel.in(MetersPerSecondPerSecond)
            / DriveConstants.kWheelRadius;

    m_io.setDriveVelocity(
        speedRadPerSec, m_driveFeedforward.calculate(speedRadPerSec, accelRadPerSec));
    m_io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    // Closed loop turn control
    m_io.setTurnPosition(new Rotation2d());

    // Open loop drive control
    Logger.recordOutput("Module/character", Timer.getFPGATimestamp());
    m_io.setDriveRawOutput(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    Logger.recordOutput("Module/stop", Timer.getFPGATimestamp());
    m_io.setDriveRawOutput(0.0);
    m_io.setTurnRawOutput(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    m_io.setDriveBrakeMode(enabled);
    m_io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return m_inputs.turnPosition;
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
  public double getDriveVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return m_inputs.drivePositionRad;
  }

  public void setCurrentLimits(double supplyLimit) {
    m_io.setCurrentLimits(supplyLimit);
  }

  public double getDriveAcceleration() {
    return m_inputs.driveAccelerationRadPerSecSq;
  }

  public double getDriveCurrent() {
    return m_inputs.driveCurrentAmps;
  }
}
