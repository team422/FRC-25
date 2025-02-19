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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private DriveIO m_driveIO;
  private SubsystemProfiles<DriveProfiles> m_profiles;
  public final DriveInputsAutoLogged m_driveInputs;

  public enum DriveProfiles {
    kIdle,
    kLeft,
    kRight,
    kForward,
    kBackward
  }

  public Drive(DriveIO io) {
    m_driveIO = io;
    m_driveInputs = new DriveInputsAutoLogged();

    HashMap<DriveProfiles, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(DriveProfiles.kIdle, this::idlePeriodic);
    periodicHash.put(DriveProfiles.kLeft, this::leftPeriodic);
    periodicHash.put(DriveProfiles.kRight, this::rightPeriodic);
    periodicHash.put(DriveProfiles.kForward, this::forwardPeriodic);
    periodicHash.put(DriveProfiles.kBackward, this::backwardPeriodic);

    m_profiles = new SubsystemProfiles<>(periodicHash, DriveProfiles.kIdle);
  }

  public void periodic() {
    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Drive/Inputs", m_driveInputs);
    Logger.recordOutput("Drive/State", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_driveIO.updateInputs(m_driveInputs);
  }

  public void leftPeriodic() {
    m_driveIO.setVoltage(
        -DriveConstants.kRotationVoltage,
        -DriveConstants.kRotationVoltage,
        DriveConstants.kRotationVoltage,
        DriveConstants.kRotationVoltage);
  }

  public void rightPeriodic() {
    m_driveIO.setVoltage(
        DriveConstants.kRotationVoltage,
        DriveConstants.kRotationVoltage,
        -DriveConstants.kRotationVoltage,
        -DriveConstants.kRotationVoltage);
  }

  public void forwardPeriodic() {
    m_driveIO.setVoltage(
        DriveConstants.kMovementVoltage,
        DriveConstants.kMovementVoltage,
        DriveConstants.kMovementVoltage,
        DriveConstants.kMovementVoltage);
  }

  public void backwardPeriodic() {
    m_driveIO.setVoltage(
        -DriveConstants.kMovementVoltage,
        -DriveConstants.kMovementVoltage,
        -DriveConstants.kMovementVoltage,
        -DriveConstants.kMovementVoltage);
  }
}
