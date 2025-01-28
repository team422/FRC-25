package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorIO;
import frc.robot.subsystems.manipulator.coralDetector.CoralDetectorInputsAutoLogged;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerIO;
import frc.robot.subsystems.manipulator.roller.ManipulatorRollerInputsAutoLogged;
import frc.robot.subsystems.manipulator.wrist.WristIO;
import frc.robot.subsystems.manipulator.wrist.WristInputsAutoLogged;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private ManipulatorRollerIO m_rollerIO;
  private WristIO m_wristIO;
  private CoralDetectorIO m_coralDetectorIO;

  private FieldConstants.ReefHeight m_desiredScoringLocation;

  public final ManipulatorRollerInputsAutoLogged m_rollerInputs =
      new ManipulatorRollerInputsAutoLogged();
  public final WristInputsAutoLogged m_wristInputs = new WristInputsAutoLogged();
  public final CoralDetectorInputsAutoLogged m_coralDetectorInputs =
      new CoralDetectorInputsAutoLogged();

  public static enum ManipulatorState {
    kStow,
    kIntaking,
    kScoring,
  }

  private SubsystemProfiles<ManipulatorState> m_profiles;

  public Manipulator(
      ManipulatorRollerIO rollerIO, WristIO wristIO, CoralDetectorIO coralDetectorIO) {
    m_rollerIO = rollerIO;
    m_wristIO = wristIO;
    m_coralDetectorIO = coralDetectorIO;

    Map<ManipulatorState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ManipulatorState.kStow, this::stowPeriodic);
    periodicHash.put(ManipulatorState.kIntaking, this::intakingPeriodic);
    periodicHash.put(ManipulatorState.kScoring, this::scoringPeriodic);
    m_profiles = new SubsystemProfiles<>(periodicHash, ManipulatorState.kStow);
  }

  @Override
  public void periodic() {
    double start = Timer.getFPGATimestamp();

    m_rollerIO.updateInputs(m_rollerInputs);
    m_wristIO.updateInputs(m_wristInputs);
    m_coralDetectorIO.updateInputs(m_coralDetectorInputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Manipulator/Roller", m_rollerInputs);
    Logger.processInputs("Manipulator/Wrist", m_wristInputs);
    Logger.processInputs("Manipulator/CoralDetector", m_coralDetectorInputs);
    Logger.recordOutput("Manipulator/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("PeriodicTime/Manipulator", Timer.getFPGATimestamp() - start);
  }

  public void updateState(ManipulatorState state) {
    m_profiles.setCurrentProfile(state);
  }

  public ManipulatorState getState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredScoringLocation(FieldConstants.ReefHeight location) {
    m_desiredScoringLocation = location;
  }

  public void stowPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.get());
  }

  public void intakingPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristIntakeAngle.get()));
    if (m_coralDetectorIO.hasGamePiece()) {
      m_rollerIO.setVoltage(0.0);
    } else {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerIntakeVoltage.get());
    }
  }

  public void scoringPeriodic() {
    // use the existing pitch of each level but add an offset (should be constant for all levels)
    // this may need to be changed if it differs in real life
    var desiredAngle =
        Rotation2d.fromDegrees(
            m_desiredScoringLocation.pitch - ManipulatorConstants.kWristScoringOffset.get());
    m_wristIO.setDesiredAngle(desiredAngle);
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerScoringVoltage.get());
  }
}
