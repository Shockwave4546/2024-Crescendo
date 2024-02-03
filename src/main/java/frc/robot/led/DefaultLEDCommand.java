package frc.robot.led;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultLEDCommand extends Command {
  private final LEDSubsystem led;

  public DefaultLEDCommand(LEDSubsystem led) {
    this.led = led;
    addRequirements(led);
  }

  @Override public void execute() {
    led.executePattern();
  }
}