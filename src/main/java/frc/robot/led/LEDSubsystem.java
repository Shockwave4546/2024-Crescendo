package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;
import frc.robot.shuffleboard.ShuffleboardBoolean;
import frc.robot.shuffleboard.ShuffleboardDouble;

import java.util.function.Consumer;

public class LEDSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("LED");
  private final ShuffleboardBoolean useManual = new ShuffleboardBoolean(tab, "Use Manual", false);
  private final ShuffleboardDouble R = new ShuffleboardDouble(tab, "R", 0.0);
  private final ShuffleboardDouble G = new ShuffleboardDouble(tab, "G", 0.0);
  private final ShuffleboardDouble B = new ShuffleboardDouble(tab, "B", 0.0);
  private final AddressableLED led = new AddressableLED(LED.PWM_ID);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED.BUFFER_LENGTH);
  private Pattern pattern = Pattern.RAINBOW;

  public LEDSubsystem() {
    led.setLength(buffer.getLength());
    led.start();
    setDefaultCommand(new DefaultLEDCommand(this));
  }

  @Override public void periodic() {
    if (useManual.get()) setPattern(Pattern.MANUAL);
    led.setData(buffer);
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

  private boolean toggle;
  public void epilipsy(Color color) {
    this.toggle = !this.toggle;

    setStaticColor(new Color(0, 0, 0));
    if (toggle) return;
    setStaticColor(color);
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
    OFF(LEDSubsystem::stopLEDs);

    private final Consumer<LEDSubsystem> action;

    Pattern(Consumer<LEDSubsystem> action) {
      this.action = action;
    }
  }
}