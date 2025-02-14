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

  public static enum LedState {
    kDisabled,
    kEnabled,
    kHasGamepiece,
    kAutoScore,
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
    if (m_state == LedState.kAlert && state != LedState.kOff) {
      // don't allow the alert state to be overridden (an alert needs to be cleared with code
      // restart)
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
      case kFullTuning:
        pattern = LEDPattern.solid(LedConstants.kFullTuning);
        break;
      case kOff:
      default:
        pattern = LEDPattern.kOff;
        break;
    }
    pattern.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }
}
