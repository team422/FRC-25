package frc.robot.subsystems.led;

import edu.wpi.first.hal.HALUtil;
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

  private static final double kMismatchFrequency = 5.0; // Hz
  private static final double kMismatchPeriod = 1.0 / kMismatchFrequency; // seconds
  private double m_lastMismatchTime = 0.0; // seconds

  private static final double kOTBL1Frequency = 5.0; // Hz
  private static final double kOTBL1Period = 1.0 / kOTBL1Frequency; // seconds
  private double m_lastOTBL1Time = 0.0;
  private boolean m_otbOn = true;

  private static final double kGamepieceFlashFrequency = 5.0; // Hz
  private static final double kGamepieceFlashPeriod = 1.0 / kGamepieceFlashFrequency; // seconds
  private double m_lastGamepieceFlashTime = 0.0;
  private boolean m_flashColor = false;
  private boolean m_gamePieceFlash = false;
  private double m_flashStartTime = 0.0;
  private static final double kFlashDuration = 2.0; // seconds

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
    kTargetMismatch,
    kSuperAlert,
    kOff,
  }

  public Led(int port, int length) {
    m_strip = new AddressableLED(port);
    m_buffer = new AddressableLEDBuffer(length);
    m_strip.setLength(length);
    m_strip.start();
  }

  @Override
  public void periodic() {
    double start = HALUtil.getFPGATime();

    if (m_state == LedState.kTargetMismatch) {
      // if the mismatch timer is running, check if it has elapsed
      targetMismatch();
    }

    if (m_gamePieceFlash) {
      if (Timer.getFPGATimestamp() - m_flashStartTime > kFlashDuration) {
        m_gamePieceFlash = false;
        m_flashColor = false;
        updateLEDState();
      } else {
        gamepieceFlash();
      }
    }

    if (m_state == LedState.kSuperAlert) {
      // override all other modifiers
      superAlert();
    }

    if (!RobotState.getInstance().getOtbRunthrough()) {
      blink();
    } else {
      if (!m_otbOn) {
        m_otbOn = true;
        updateLEDState();
      }
    }

    Logger.recordOutput("LED/State", m_state.toString());
    Logger.recordOutput("LED/Color", m_buffer.getLED(0).toString());
    Logger.recordOutput("LED/OtbOn", m_otbOn);
    Logger.recordOutput("LED/GamepieceFlash", m_gamePieceFlash);
    Logger.recordOutput("LED/FlashColor", m_flashColor);

    Logger.recordOutput("PeriodicTime/LED", (HALUtil.getFPGATime() - start) / 1000.0);
  }

  public void updateState(LedState state) {
    if (m_state == state) {
      // exit early so we don't have to update
      return;
    }
    if (m_state == LedState.kSuperAlert) {
      // super alert must be cleared with code restart
      return;
    }
    if (m_state == LedState.kFullTuning && state != LedState.kSuperAlert) {
      // don't allow full tuning to be cancelled except by super alert
      return;
    }
    if (m_state == LedState.kAlert
        && state != LedState.kFullTuning
        && state != LedState.kSuperAlert) {
      // only allow full tuning and super alert to cancel alert
      return;
    }
    m_state = state;

    if (m_otbOn && !m_flashColor) {
      updateLEDState();
    }
  }

  public LedState getCurrentState() {
    return m_state;
  }

  public void cancelAlert() {
    if (m_state == LedState.kAlert) {
      m_state = LedState.kL1;
    }
  }

  private void superAlert() {
    // rapidly flash red
    if (Timer.getFPGATimestamp() % 0.35 < 0.175) {
      // flash on
      LEDPattern.solid(convertColor(Color.kRed)).applyTo(m_buffer);
    } else {
      // flash off
      LEDPattern.kOff.applyTo(m_buffer);
    }
    m_strip.setData(m_buffer);
  }

  private void targetMismatch() {
    if (Timer.getFPGATimestamp() - m_lastMismatchTime >= kMismatchPeriod) {
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

      m_lastMismatchTime = Timer.getFPGATimestamp();
    }
  }

  private void blink() {
    if (Timer.getFPGATimestamp() - m_lastOTBL1Time >= kOTBL1Period) {
      m_lastOTBL1Time = Timer.getFPGATimestamp();

      m_otbOn = !m_otbOn;
      if (m_otbOn) {
        updateLEDState();
      } else if (m_flashColor) {
        LEDPattern.solid(LedConstants.kHasGamePiece).applyTo(m_buffer);
        if (!RobotState.getInstance().getUsingVision()) {
          for (int i = 9; i <= 15; i++) {
            m_buffer.setLED(i, convertColor(LedConstants.kVisionOff));
          }
        }
        m_strip.setData(m_buffer);
      } else {
        LEDPattern.kOff.applyTo(m_buffer);
        m_strip.setData(m_buffer);
      }
    }
  }

  private void gamepieceFlash() {
    if (Timer.getFPGATimestamp() - m_lastGamepieceFlashTime >= kGamepieceFlashPeriod) {
      m_lastGamepieceFlashTime = Timer.getFPGATimestamp();

      m_flashColor = !m_flashColor;
      if (m_flashColor) {
        LEDPattern.solid(LedConstants.kHasGamePiece).applyTo(m_buffer);
        if (!RobotState.getInstance().getUsingVision()) {
          for (int i = 9; i <= 15; i++) {
            m_buffer.setLED(i, convertColor(LedConstants.kVisionOff));
          }
        }
        m_strip.setData(m_buffer);
      } else {
        updateLEDState();
      }
    }
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
      case kTargetMismatch:
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

  public void gamepiece() {
    m_flashStartTime = Timer.getFPGATimestamp();
    m_gamePieceFlash = true;
    m_flashColor = m_otbOn;
    m_lastGamepieceFlashTime = m_lastOTBL1Time;
  }

  private static Color convertColor(Color color) {
    if (RobotBase.isSimulation()) {
      return color;
    }
    // for some reason green and red are swapped on the LEDs
    return new Color(color.green, color.red, color.blue);
  }
}
