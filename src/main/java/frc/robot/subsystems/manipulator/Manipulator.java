package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Robot;
import frc.robot.subsystems.manipulator.rollers.RollerIO;
import frc.robot.subsystems.manipulator.rollers.RollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.wrist.WristIO;
import frc.robot.subsystems.manipulator.wrist.WristInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private RollerIO m_rollerIO;
  private WristIO m_wristIO;
  private SubsystemProfiles<ManipulatorState> m_profiles;
  private RollerInputsAutoLogged m_rollerInputs;
  private WristInputsAutoLogged m_wristInputs;

  public enum ManipulatorState {
    kIdle,
    kScoring,
    kOuttaking
  }

  public Manipulator(RollerIO roller, WristIO wrist) {
    m_rollerIO = roller;
    m_wristIO = wrist;

    m_wristInputs = new WristInputsAutoLogged();
    m_rollerInputs = new RollerInputsAutoLogged();

    if (RobotBase.isReal()) {
      m_wristIO.setPIDFF(
          0,
          ManipulatorConstants.kWristP.getAsDouble(),
          ManipulatorConstants.kWristI.getAsDouble(),
          ManipulatorConstants.kWristD.getAsDouble(),
          ManipulatorConstants.kWristKS.getAsDouble());
    } else {
      m_wristIO.setPIDFF(
          0,
          ManipulatorConstants.kSimWristP,
          ManipulatorConstants.kSimWristI,
          ManipulatorConstants.kSimWristD,
          0);
    }

    HashMap<ManipulatorState, Runnable> m_hash = new HashMap<>();
    m_hash.put(ManipulatorState.kIdle, this::idlePeriodic);
    m_hash.put(ManipulatorState.kScoring, this::scoringPeriodic);
    m_hash.put(ManipulatorState.kOuttaking, this::outtakingPeriodic);

    m_profiles = new SubsystemProfiles<>(m_hash, ManipulatorState.kIdle);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          if (Robot.isReal()) {
            m_wristIO.setPIDFF(
                0,
                ManipulatorConstants.kWristP.getAsDouble(),
                ManipulatorConstants.kWristI.getAsDouble(),
                ManipulatorConstants.kWristD.getAsDouble(),
                ManipulatorConstants.kWristKS.getAsDouble());
          }
        },
        ManipulatorConstants.kWristP,
        ManipulatorConstants.kWristI,
        ManipulatorConstants.kWristD,
        ManipulatorConstants.kWristKS);

    m_rollerIO.updateInputs(m_rollerInputs);
    m_wristIO.updateInputs(m_wristInputs);
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.processInputs("Manipulator/Roller", m_rollerInputs);
    Logger.processInputs("Manipulator/Wrist", m_wristInputs);
    Logger.recordOutput("Manipulator/state", m_profiles.getCurrentProfile());
  }

  private void idlePeriodic() {
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.getAsDouble());
    m_wristIO.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.getAsDouble()));
  }

  private void scoringPeriodic() {
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.getAsDouble());
    m_wristIO.setAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.getAsDouble()));
  }

  private void outtakingPeriodic() {
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerEjectVoltage.getAsDouble());
  }

  public void updateState(ManipulatorState state) {
    m_profiles.setCurrentProfile(state);
  }

  public ManipulatorState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void setAngle(Rotation2d angle) {
    m_wristIO.setAngle(angle);
  }

  public boolean atSetpoint() {
    return m_wristInputs.atSetpoint;
  }
}
