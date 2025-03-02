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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {

  private DriveCommands() {}

  public static Command arcadeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          drive.arcadeDrive(xSupplier.getAsDouble(), omegaSupplier.getAsDouble());
        },
        drive);
  }
}
