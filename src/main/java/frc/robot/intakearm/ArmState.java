package frc.robot.intakearm;

/**
 * @param name the name of the configuration.
 * @param angle in degrees.
 */
public record ArmState(String name, double angle) {
  public static final ArmState HOME = new ArmState("Home", 5.0);
  public static final ArmState MIDDLE = new ArmState("Middle", 50.0);
  public static final ArmState FLOOR = new ArmState("Floor", 195.0);
}