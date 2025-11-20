package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_pigeon = new Pigeon2(Ports.kPigeon, Ports.kDriveCanivoreName);
  private final StatusSignal<Angle> m_yaw = m_pigeon.getYaw();
  private final StatusSignal<Angle> m_pitch = m_pigeon.getPitch();
  private final StatusSignal<Angle> m_roll = m_pigeon.getRoll();
  private final Queue<Double> m_yawPositionQueue;
  private final Queue<Double> m_pitchPositionQueue;
  private final Queue<Double> m_rollPositionQueue;
  private final Queue<Double> m_timestampQueue;
  private final StatusSignal<AngularVelocity> m_yawVelocity = m_pigeon.getAngularVelocityZWorld();
  private final StatusSignal<AngularVelocity> m_pitchVelocity = m_pigeon.getAngularVelocityXWorld();
  private final StatusSignal<AngularVelocity> m_rollVelocity = m_pigeon.getAngularVelocityYWorld();
  private final StatusSignal<LinearAcceleration> m_xAcceleration = m_pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> m_yAcceleration = m_pigeon.getAccelerationY();
  private final StatusSignal<LinearAcceleration> m_zAcceleration = m_pigeon.getAccelerationZ();
  private final StatusSignal<Voltage> m_pigeonSupplyVoltage = m_pigeon.getSupplyVoltage();

  public GyroIOPigeon2() {
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
    m_pigeon.getConfigurator().setYaw(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.kOdometryFrequency,
        m_yaw,
        m_pitch,
        m_roll,
        m_yawVelocity,
        m_pitchVelocity,
        m_rollVelocity,
        m_xAcceleration,
        m_yAcceleration,
        m_zAcceleration);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, m_pigeonSupplyVoltage);

    // m_pigeon.optimizeBusUtilization();

    m_timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getYaw());
    m_pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getPitch());
    m_rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_pigeon.getRoll());

    if (Constants.kUseBaseRefreshManager) {
      CtreBaseRefreshManager.addSignals(
          List.of(
              m_yaw,
              m_pitch,
              m_roll,
              m_yawVelocity,
              m_pitchVelocity,
              m_rollVelocity,
              m_pigeonSupplyVoltage));
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
          m_yaw,
          m_pitch,
          m_roll,
          m_yawVelocity,
          m_pitchVelocity,
          m_rollVelocity,
          m_xAcceleration,
          m_yAcceleration,
          m_zAcceleration,
          m_pigeonSupplyVoltage);
    }

    // this is a hack because the pigeon doesn't have a connected motor we can check (obviously)
    // so basically if we're in sim, the supply voltage is EXACTLY zero
    // if we connected it'll be something else
    inputs.connected = m_pigeonSupplyVoltage.getValueAsDouble() != 0.0;

    if (RobotBase.isSimulation()) {
      inputs.connected = false;
    }

    inputs.yawPosition = new Rotation2d(m_yaw.getValue());
    inputs.pitchPosition = new Rotation2d(m_pitch.getValue());
    inputs.rollPosition = new Rotation2d(m_roll.getValue());
    inputs.yawVelocityRadPerSec = m_yawVelocity.getValue().in(RadiansPerSecond);
    inputs.pitchVelocityRadPerSec = m_pitchVelocity.getValue().in(RadiansPerSecond);
    inputs.rollVelocityRadPerSec = m_rollVelocity.getValue().in(RadiansPerSecond);
    inputs.xAcceleration = m_xAcceleration.getValue().in(MetersPerSecondPerSecond);
    inputs.yAcceleration = m_yAcceleration.getValue().in(MetersPerSecondPerSecond);
    inputs.zAcceleration = m_zAcceleration.getValue().in(MetersPerSecondPerSecond);

    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryPitchPositions =
        m_pitchPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryRollPositions =
        m_rollPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    m_timestampQueue.clear();
    m_yawPositionQueue.clear();
    m_pitchPositionQueue.clear();
    m_rollPositionQueue.clear();
  }
}
