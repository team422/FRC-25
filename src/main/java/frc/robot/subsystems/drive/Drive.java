package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    m_io = io;

    m_drivetrain = new DifferentialDrive(m_io.getMotor(0), m_io.getMotor(2));

    configureMotors();

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

  public void configureMotors() {
    SparkMaxConfig m_frontConfig = new SparkMaxConfig();
    m_frontConfig.idleMode(IdleMode.kCoast);
    m_io.getMotor(0)
        .configure(
            m_frontConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters); // front left
    m_io.getMotor(2)
        .configure(
            m_frontConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters); // front right

    SparkMaxConfig m_LRConfig = new SparkMaxConfig();
    m_LRConfig.idleMode(IdleMode.kCoast);
    m_LRConfig.follow(m_io.getMotor(0), true);
    m_io.getMotor(1)
        .configure(m_LRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig m_RRConfig = new SparkMaxConfig();
    m_RRConfig.idleMode(IdleMode.kCoast);
    m_RRConfig.follow(m_io.getMotor(2), true);
    m_io.getMotor(3)
        .configure(m_RRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
