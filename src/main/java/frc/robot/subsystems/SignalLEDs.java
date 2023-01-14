// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class SignalLEDs extends SubsystemBase {

  public static final int kLedLength = 13;
  public static final int kLedPwmPort = 3;
  // Must be a PWM header, not MXP or DIO
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  public enum mode {
    CUBE,
    CONE,
    OFF,
  };

  private mode m_mode;

  /** Creates a new SignalLEDs. */
  public SignalLEDs() {
    m_led = new AddressableLED(3);
    m_ledBuffer = new AddressableLEDBuffer(kLedLength);
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(m_ledBuffer.getLength());
    // Both LED strips MUST Be the same length
    m_mode = mode.CUBE;
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SolidColor();
  }

  private void SolidColor() {
    for (var i=0; i<m_ledBuffer.getLength(); i++) {
      switch(m_mode) {
        case CUBE:
          m_ledBuffer.setRGB(i, 192, 0, 192);
          break;
        case CONE:
          m_ledBuffer.setRGB(i, 255, 255, 0);
          break;
        case OFF:
          m_ledBuffer.setRGB(i, 0, 0, 0);
          break;
      }
      m_led.setData(m_ledBuffer);
    }
  }
  public void Disabled() {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, 255, 48, 0);
      }
      m_led.setData(m_ledBuffer);
    }    

  public void setMode(mode newMode) {
    m_mode = newMode;
  }
}
