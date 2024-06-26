package org.dovershockwave.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.Constants.LED;
import org.dovershockwave.shuffleboard.ShuffleboardBoolean;
import org.dovershockwave.shuffleboard.ShuffleboardDouble;

import java.util.function.Consumer;

public class LEDSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("LED");
  private final ShuffleboardBoolean useManual = new ShuffleboardBoolean(tab, "Use Manual", false);
  private final ShuffleboardDouble R = new ShuffleboardDouble(tab, "R", 0.0);
  private final ShuffleboardDouble G = new ShuffleboardDouble(tab, "G", 0.0);
  private final ShuffleboardDouble B = new ShuffleboardDouble(tab, "B", 0.0);
  private final AddressableLED led = new AddressableLED(LED.PWM_ID);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED.BUFFER_LENGTH);
  private Pattern pattern = Pattern.WHITE;

  public LEDSubsystem() {
    led.setLength(buffer.getLength());
    led.start();
    setDefaultCommand(new DefaultLEDCommand(this));
  }

  @Override public void periodic() {
    if (useManual.get()) setPattern(Pattern.MANUAL);
    led.setData(buffer);
  }

  public void greenAndWhiteChase() {
    for (var i = 0; i < buffer.getLength(); i++) {
      if (i % 2 == 0) {
        buffer.setHSV(i, 120, 255, 128); // Set green color for even-indexed LEDs
      } else {
        buffer.setHSV(i, 0, 255, 128); // Set white color for odd-indexed LEDs
      }
    }
  }

  /**
   * Source: <a href="https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html">...</a>
   */
  private int rainbowFirstPixelHue;
  public void rainbow() {
    for (var i = 0; i < buffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  public void stopLEDs() {
    setStaticColor(LED.OFF);
    led.stop();
  }

  public void setStaticColor(Color color) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, color);
    }
  }

  public void useManualColor() {
    setStaticColor(new Color(R.get(), G.get(), B.get()));
  }

  public void executePattern() {
    pattern.action.accept(this);
  }

  public void setPattern(Pattern pattern) {
    this.pattern = pattern;
  }

  public enum Pattern {
    MANUAL(LEDSubsystem::useManualColor),
    WHITE((led) -> led.setStaticColor(LED.WHITE)),
    RAINBOW(LEDSubsystem::rainbow),
    RED((led) -> led.setStaticColor(LED.RED)),
    GREEN((led) -> led.setStaticColor(LED.GREEN)),
    WHITE_GREEN_CHASE(LEDSubsystem::greenAndWhiteChase),
    OFF(LEDSubsystem::stopLEDs);

    private final Consumer<LEDSubsystem> action;

    Pattern(Consumer<LEDSubsystem> action) {
      this.action = action;
    }
  }
}