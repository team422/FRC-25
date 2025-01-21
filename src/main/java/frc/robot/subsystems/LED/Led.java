package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private AddressableLED m_strip;
  private AddressableLEDBuffer m_buffer;
  private LedState m_state = LedState.kOff;
  private LedState m_lastState = LedState.kOff;

  public static enum LedState {
    kDisabled,
    kEnabled,
    kHasGamepiece,
    kAutoScore,
    kAlert,
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
    if (m_state == m_lastState) {
      updateLEDState();
      m_lastState = m_state;
    }
  }

  public void setState(LedState state) {
    m_state = state;
  }

  public void updateLEDState() {
    LEDPattern pattern;
    switch (m_state) {
      case kDisabled:
        pattern = LEDPattern.solid(LedConstants.kDisabled);
        break;
      case kEnabled:
        pattern = LEDPattern.solid(LedConstants.kEnabled);
        break;
      case kHasGamepiece:
        pattern = LEDPattern.solid(LedConstants.kHasGampiece);
        break;
      case kAutoScore:
        pattern = LEDPattern.solid(LedConstants.kAutoscore);
        break;
      case kAlert:
        pattern = LEDPattern.solid(LedConstants.kAlert);
        break;
      case kOff:
      default:
        pattern = LEDPattern.solid(LedConstants.kOff);
        break;
    }
    pattern.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }
}
