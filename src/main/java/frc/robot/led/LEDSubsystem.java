package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(LED.PWM_ID);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED.BUFFER_LENGTH);
  private int rainbowFirstPixelHue;

  public LEDSubsystem() {
    led.setLength(buffer.getLength());
    led.start();
    setDefaultCommand(new DefaultLEDCommand(this));
  }

  @Override public void periodic() {
    led.setData(buffer);
  }

  /**
   * Source: <a href="https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html">...</a>
   */
  public void rainbow() {
    for (var i = 0; i < buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  public void stopLEDs() {
    led.stop();
  }

  public void setStaticColor(Color color) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }
}