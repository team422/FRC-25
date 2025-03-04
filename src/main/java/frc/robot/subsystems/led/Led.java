package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private AddressableLED m_strip;
  private AddressableLEDBuffer m_buffer;
  private LedState m_state = LedState.kOff;

  public static enum LedState {
    kDisabled,
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

  private void updateLEDState() {
    LEDPattern pattern;
    switch (m_state) {
      case kDisabled:
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
    pattern.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }

  private static Color convertColor(Color color) {
    // for some reason green and red are swapped on the LEDs
    return new Color(color.green, color.red, color.blue);
  }
}
