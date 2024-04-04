package org.dovershockwave.shooterwrist;

public record WristState(String name, double angle) {
  public static final WristState STARTING = new WristState("Starting", 2.5);
  public static final WristState HOME = new WristState("Home", 40.0);
  public static final WristState SUBWOOFER = new WristState("Subwoofer", 30.0);
  public static final WristState SPIT = new WristState("Spit", 30.0);
  public static final WristState AMP = new WristState("Amp", 25.0);
  public static final WristState INTERPOLATED = new WristState("Interpolated", -1.0);
}
