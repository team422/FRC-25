package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private AddressableLED m_strip;
  private AddressableLEDBuffer m_buffer;
  private LedState m_state = LedState.kOff;

  public static enum LedState {
    kLocationCheck,
    kL1,
    kL2,
    kL3,
    kL4,
    kAlert,
    kFullTuning,
    kOff
  }

  public Led(int port, int length) {
    m_strip = new AddressableLED(port);
    m_buffer = new AddressableLEDBuffer(length);
    m_strip.setLength(length);
    m_strip.start();
  }

  @Override
  public void periodic() {
    // TODO: this does not work because the start pose isn't rotated to match alliance
    // if (m_state == LedState.kLocationCheck) {
    //   locationCheck();
    // }

    Logger.recordOutput("LED/State", m_state.toString());
    Logger.recordOutput("LED/Color", m_buffer.getLED(0).toString());
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
  }

  public LedState getCurrentState() {
    return m_state;
  }

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

  private void updateLEDState() {
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
        pattern = LEDPattern.solid(convertColor(LedConstants.kAlert));
        break;
      case kFullTuning:
        pattern = LEDPattern.solid(convertColor(LedConstants.kFullTuning));
        break;
      case kOff:
      default:
        pattern = LEDPattern.kOff;
        break;
    }
    if (pattern != null) {
      pattern.applyTo(m_buffer);
    }
    m_strip.setData(m_buffer);
  }

  private static Color convertColor(Color color) {
    // for some reason green and red are swapped on the LEDs
    return new Color(color.green, color.red, color.blue);
  }
}
