package org.dovershockwave.shooterwrist;

public record WristState(String name, double angle) {
  public static final WristState STARTNG = new WristState("Starting", 2.5);
  public static final WristState HOME = new WristState("Home", 30.0);
}
