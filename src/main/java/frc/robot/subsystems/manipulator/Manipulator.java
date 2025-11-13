package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.manipulator.rollers.RollerIO;
import frc.robot.subsystems.manipulator.rollers.RollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.wrist.WristIO;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private RollerIO m_rollerIO;
  private WristIO m_wristIO;
  private SubsystemProfiles<ManipulatorState> m_profiles;
  private RollerInputsAutoLogged m_inputs;

  public enum ManipulatorState {
    kIdle,
    kScoring,
    kOuttaking
  }

  public Manipulator(RollerIO roller, WristIO wrist) {
    m_rollerIO = roller;
    m_wristIO = wrist;

    HashMap<ManipulatorState, Runnable> m_hash = new HashMap<>();
    m_hash.put(ManipulatorState.kIdle, this::idlePeriodic);
    m_hash.put(ManipulatorState.kScoring, this::scoringPeriodic);
    m_hash.put(ManipulatorState.kOuttaking, this::outtakingPeriodic);

    m_profiles = new SubsystemProfiles<>(m_hash, ManipulatorState.kIdle);
  }

  @Override
  public void periodic() {
    m_rollerIO.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.processInputs("Manipulator", m_inputs);
    Logger.recordOutput("Manipulator/state", m_profiles.getCurrentProfile());
  }

  private void idlePeriodic() {
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.getAsDouble());
  }

  private void scoringPeriodic() {
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.getAsDouble());
  }

  private void outtakingPeriodic() {
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerEjectVoltage.getAsDouble());
  }

  public void updateState(ManipulatorState state) {
    m_profiles.setCurrentProfile(state);
  }
}
