package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private AddressableLED m_strip;
  private AddressableLEDBuffer m_buffer;
  private LedState m_state = LedState.kOff;

  private Timer m_timer = new Timer();
  private static final double kCrazyTurnRate = 5.0; // hz
  private static final double kCrazyTurnInterval = 1.0 / kCrazyTurnRate; // seconds
  private double m_lastCallTime = 0.0; // seconds

  public static enum LedState {
    kLocationCheck,
    kL1,
    kL2,
    kL3,
    kL4,
    kAlert,
    kFullTuning,
    kAutoscoreMeasurementsBad,
    kAutoscoreMeasurementsGood,
    kCrazyTurn,
    kOff,
  }

  public Led(int port, int length) {
    m_strip = new AddressableLED(port);
    m_buffer = new AddressableLEDBuffer(length);
    m_strip.setLength(length);
    m_strip.start();

    m_timer.start();
  }

  @Override
  public void periodic() {
    // unnecessary for now but i dont wanna delete the math i did just yet
    // if (m_state == LedState.kLocationCheck) {
    //   locationCheck();
    // }

    if (RobotState.getInstance().getCrazyTurn()) {
      updateState(LedState.kCrazyTurn);
    }

    if (m_state == LedState.kCrazyTurn && m_timer.get() - m_lastCallTime > kCrazyTurnInterval) {
      crazyTurn();
      m_lastCallTime = m_timer.get();
    }

    Logger.recordOutput("LED/State", m_state.toString());
    Logger.recordOutput("LED/Color", m_buffer.getLED(0).toString());
    Logger.recordOutput("LED/Time", m_timer.get() - m_lastCallTime);
  }

  public void updateState(LedState state) {
    if (m_state == state) {
      // exit early so we don't have to update
      return;
    }
    if (m_state == LedState.kFullTuning) {
      // don't allow full tuning to be cancelled
      return;
    }
    if (m_state == LedState.kAlert && state != LedState.kFullTuning) {
      // don't allow the alert state to be overridden (an alert needs to be cleared with code
      // restart)
      // but full tuning can override it
      return;
    }
    m_state = state;
    updateLEDState();

    m_timer.restart();
  }

  public LedState getCurrentState() {
    return m_state;
  }

  public void cancelAlert() {
    if (m_state == LedState.kAlert) {
      m_state = LedState.kL1;
    }
  }

  @SuppressWarnings("unused")
  private void locationCheck() {
    Pose2d robotPose = RobotState.getInstance().getRobotPose();
    Pose2d startPose = RobotState.getInstance().getPathPlannerStartPose();
    double yawError = robotPose.getRotation().minus(startPose.getRotation()).getDegrees();
    int numYawLeds = (int) Math.round(yawError);
    numYawLeds = MathUtil.clamp(numYawLeds, 0, m_buffer.getLength() / 2);
    double distanceError =
        Units.metersToInches(robotPose.getTranslation().getDistance(startPose.getTranslation()));
    int numDistanceLeds = (int) Math.round(distanceError);
    numDistanceLeds = MathUtil.clamp(numDistanceLeds, 0, m_buffer.getLength() / 2);
    // left side is yaw error, right side is distance error
    // start in middle and work way out

    int mid = m_buffer.getLength() / 2; // Middle index

    // Place 1s
    for (int i = 0; i < numYawLeds; i++) {
      m_buffer.setLED(mid - (numYawLeds - i), LedConstants.kLocationCheckYaw);
    }

    // Place 2s
    for (int i = 0; i < numDistanceLeds; i++) {
      m_buffer.setLED(mid + i, LedConstants.kLocationCheckDistance);
    }

    Logger.recordOutput("LED/LocationCheck/YawError", yawError);
    Logger.recordOutput("LED/LocationCheck/DistanceError", distanceError);
    Logger.recordOutput("LED/LocationCheck/NumYawLeds", numYawLeds);
    Logger.recordOutput("LED/LocationCheck/NumDistanceLeds", numDistanceLeds);
  }

  private void crazyTurn() {
    // random colors
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setLED(
          i,
          new Color(
              (int) (Math.random() * 255),
              (int) (Math.random() * 255),
              (int) (Math.random() * 255)));
    }

    m_strip.setData(m_buffer);
  }

  public void updateLEDState() {
    LEDPattern pattern;
    switch (m_state) {
      case kLocationCheck:
        // pattern = null;
        pattern = LEDPattern.solid(convertColor(LedConstants.kDisabled));
        break;
      case kL1:
        pattern = LEDPattern.solid(convertColor(LedConstants.kL1));
        break;
      case kL2:
        pattern = LEDPattern.solid(convertColor(LedConstants.kL2));
        break;
      case kL3:
        pattern = LEDPattern.solid(convertColor(LedConstants.kL3));
        break;
      case kL4:
        pattern = LEDPattern.solid(convertColor(LedConstants.kL4));
        break;
      case kAlert:
        pattern = LEDPattern.rainbow(255, 255);
        break;
      case kFullTuning:
        pattern = LEDPattern.solid(convertColor(LedConstants.kFullTuning));
        break;
      case kAutoscoreMeasurementsBad:
        pattern = LEDPattern.solid(convertColor(LedConstants.kAutoscoreMeasurementsBad));
        break;
      case kAutoscoreMeasurementsGood:
        pattern = LEDPattern.solid(convertColor(LedConstants.kAutoscoreMeasurementsGood));
        break;
      case kCrazyTurn:
        pattern = null;
        break;
      case kOff:
      default:
        pattern = LEDPattern.kOff;
        break;
    }
    if (pattern != null) {
      pattern.applyTo(m_buffer);
    }
    if (!RobotState.getInstance().getUsingVision()) {
      for (int i = 9; i <= 15; i++) {
        m_buffer.setLED(i, convertColor(LedConstants.kVisionOff));
      }
    }
    m_strip.setData(m_buffer);
  }

  private static Color convertColor(Color color) {
    if (RobotBase.isSimulation()) {
      return color;
    }
    // for some reason green and red are swapped on the LEDs
    return new Color(color.green, color.red, color.blue);
  }
}
