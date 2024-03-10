package org.dovershockwave.shooter;

public record ShooterState(String name, double bottomRPS, double topRPS) {
  public static final ShooterState STOPPED = new ShooterState("Stopped", 0.0, 0.0);
  public static final ShooterState AMP = new ShooterState("Amp", 22.5, 22.5);
  public static final ShooterState SUBWOOFER = new ShooterState("Subwoofer", 70.0, 30.0);
  public static final ShooterState INTERPOLATED = new ShooterState("Interpolated", -1.0, -1.0);
}