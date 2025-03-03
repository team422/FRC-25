package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private DifferentialDrive m_drivetrain;

  private SubsystemProfiles<DriveProfiles> m_profiles;
  public final DriveInputsAutoLogged m_driveInputs;

  private DriveIO m_io;

  public enum DriveProfiles {
    kDefault
  }

  public Drive(DriveIO io) {

    m_drivetrain = new DifferentialDrive(m_io::setLeft, m_io::setRight);

    m_driveInputs = new DriveInputsAutoLogged();

    HashMap<DriveProfiles, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(DriveProfiles.kDefault, this::defaultPeriodic);
    m_profiles = new SubsystemProfiles<>(periodicHash, DriveProfiles.kDefault);
  }

  public void periodic() {
    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Drive/Inputs", m_driveInputs);
    Logger.recordOutput("Drive/State", m_profiles.getCurrentProfile());
    m_io.updateInputs(m_driveInputs);
  }

  public void defaultPeriodic() {}

  public void arcadeDrive(double x, double omega) {
    m_drivetrain.arcadeDrive(x, omega);
  }
}
