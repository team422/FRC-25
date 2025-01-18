package frc.robot.subsystems.LED;

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
  }

  public void setSolidColor(Color color) {
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setLED(i, color);
    }
    m_strip.setData(m_buffer);
  }

  public void DisabledPeriodic() {
    LEDPattern red = LEDPattern.solid(LedConstants.kDisabled);
    red.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }

  public void EnabledPeriodic() {
    LEDPattern yellow = LEDPattern.solid(LedConstants.kEnabled);
    yellow.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }

  public void HasGamepiecePeriodic() {
    LEDPattern darkMagenta = LEDPattern.solid(LedConstants.kHasGampiece);
    darkMagenta.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }

  public void AutoscorePeriodic() {
    LEDPattern green = LEDPattern.solid(LedConstants.kAutoscore);
    green.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }

  public void AlertPeriodic() {
    LEDPattern gold = LEDPattern.solid(LedConstants.kAlert);
    gold.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }

  public void OFFPeriodic() {
    LEDPattern red = LEDPattern.solid(LedConstants.kOff);
    red.applyTo(m_buffer);
    m_strip.setData(m_buffer);
  }
}
