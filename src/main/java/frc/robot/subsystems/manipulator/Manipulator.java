package frc.robot.subsystems.manipulator;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.littletonUtils.LoggedTunableNumber;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
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

  private Rotation2d m_desiredWristAngle = new Rotation2d();
  private boolean m_runRollerScoring = false;

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
    double start = HALUtil.getFPGATime();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_wristIO.setPIDFF(
              ManipulatorConstants.kWristP.get(),
              ManipulatorConstants.kWristI.get(),
              ManipulatorConstants.kWristD.get(),
              ManipulatorConstants.kWristKS.get(),
              ManipulatorConstants.kWristKG.get());
        },
        ManipulatorConstants.kWristP,
        ManipulatorConstants.kWristI,
        ManipulatorConstants.kWristD,
        ManipulatorConstants.kWristKS,
        ManipulatorConstants.kWristKG);

    m_rollerIO.updateInputs(m_rollerInputs);
    m_wristIO.updateInputs(m_wristInputs);
    m_coralDetectorIO.updateInputs(m_coralDetectorInputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Manipulator/Roller", m_rollerInputs);
    Logger.processInputs("Manipulator/Wrist", m_wristInputs);
    Logger.processInputs("Manipulator/CoralDetector", m_coralDetectorInputs);
    Logger.recordOutput("Manipulator/State", m_profiles.getCurrentProfile());
    Logger.recordOutput("PeriodicTime/Manipulator", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(ManipulatorState state) {
    // TODO: this makes sense but i have the feeling it will cause an issue during testing
    // DON'T FORGET ABOUT THIS
    m_runRollerScoring = false;

    m_profiles.setCurrentProfile(state);
  }

  public ManipulatorState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public void setDesiredWristAngle(Rotation2d angle) {
    m_desiredWristAngle = angle;
  }

  public Rotation2d getDesiredWristAngle() {
    return m_desiredWristAngle;
  }

  public void stowPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristStowAngle.get()));
    m_rollerIO.setVoltage(ManipulatorConstants.kRollerStowVoltage.get());
  }

  public void intakingPeriodic() {
    m_wristIO.setDesiredAngle(Rotation2d.fromDegrees(ManipulatorConstants.kWristIntakeAngle.get()));
    // if we have a game piece stop
    // if the indexer isn't running then the elevator and manipulator aren't both in position yet
    // so we don't want to intake yet
    if (m_coralDetectorIO.hasGamePiece()
        || RobotState.getInstance().getIndexerState() != IndexerState.kIndexing) {
      m_rollerIO.setVoltage(0.0);
    } else {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerIntakeVoltage.get());
    }
  }

  public void scoringPeriodic() {
    // use the existing pitch of each level but add an offset (should be constant for all levels)
    // this may need to be changed if it differs in real life
    m_wristIO.setDesiredAngle(m_desiredWristAngle);
    if (m_runRollerScoring) {
      m_rollerIO.setVoltage(ManipulatorConstants.kRollerScoringVoltage.get());
    } else {
      m_rollerIO.setVoltage(0.0);
    }
  }

  public void runRollerScoring() {
    m_runRollerScoring = true;
  }

  public boolean atSetpoint() {
    return m_wristIO.atSetpoint();
  }

  public boolean hasGamePiece() {
    return m_coralDetectorIO.hasGamePiece();
  }

  public Rotation2d getCurrAngle() {
    return m_wristIO.getCurrAngle();
  }
}
