package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class ModuleIOReplay implements ModuleIO {

  @Override
  public void updateInputs(ModuleIOInputs inputs) {}

  @Override
  public void setDriveVelocity(double velocity, double feedforward) {}

  @Override
  public void setTurnPosition(Rotation2d position) {}

  @Override
  public void setDriveBrakeMode(boolean enable) {}

  @Override
  public void setTurnBrakeMode(boolean enable) {}

  @Override
  public void setCurrentLimits(double supplyLimit) {}

  @Override
  public void setDriveRawOutput(double output) {}

  @Override
  public void setTurnRawOutput(double output) {}

  @Override
  public void setDrivePID(double kP, double kI, double kD) {}

  @Override
  public void setTurnPID(double kP, double kI, double kD) {}

  @Override
  public void resetTurnMotor(Angle position) {}
}
