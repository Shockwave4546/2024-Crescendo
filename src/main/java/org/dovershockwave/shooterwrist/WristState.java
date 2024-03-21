package org.dovershockwave.shooterwrist;

public record WristState(String name, double angle) {
  public static final WristState HOME = new WristState("Home", 0.0);
}
