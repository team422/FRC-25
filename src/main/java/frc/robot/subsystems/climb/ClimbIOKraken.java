package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOKraken implements ClimbIO {
  private TalonFX m_motor;

  private Slot0Configs m_Slot0Config = new Slot0Configs();
  private VoltageOut m_voltageOut = new VoltageOut(0);
  private PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private Rotation2d m_desiredAngle = Rotation2d.fromDegrees(0);
  private boolean m_positionControl = true;

  public ClimbIOKraken(int port) {
    m_motor = new TalonFX(port);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;

    m_motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ClimbInputs inputs) {
    inputs.currPositionRad = Units.rotationsToRadians(m_motor.getPosition().getValueAsDouble());
    inputs.desiredPositionRad = m_desiredAngle.getRadians();
    inputs.atSetpoint = Math.abs(m_desiredAngle.getRadians() - inputs.currPositionRad) < ClimbConstants.kClimbTolerance.getRadians();
    inputs.positionControl = m_positionControl;
    inputs.voltage = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.current = m_motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS, double kV) {
    m_Slot0Config.kP = kP;
    m_Slot0Config.kI = kI;
    m_Slot0Config.kD = kD;
    m_Slot0Config.kS = kS;
    m_Slot0Config.kV = kV;

    m_motor.getConfigurator().apply(m_Slot0Config);
  }

  @Override
  public void setVoltage(double voltage) {
    m_positionControl = false;
    m_desiredAngle = Rotation2d.fromRadians(0);
    m_motor.setControl(m_voltageOut.withOutput(voltage));
  }

  @Override
  public void setDesiredAngle(Rotation2d angle) {
    m_positionControl = true;
    m_desiredAngle = angle;
    m_motor.setControl(m_positionVoltage.withPosition(Units.radiansToRotations(angle.getRadians())));
  }
  
}
