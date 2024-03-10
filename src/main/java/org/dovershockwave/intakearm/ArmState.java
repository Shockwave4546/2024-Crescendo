package org.dovershockwave.intakearm;

/**
 * @param name the name of the configuration.
 * @param angle in degrees.
 */
public record ArmState(String name, double angle) {
  public static final ArmState HOME = new ArmState("Home", 5.0);
  public static final ArmState FLOOR = new ArmState("Floor", 195.0);
}