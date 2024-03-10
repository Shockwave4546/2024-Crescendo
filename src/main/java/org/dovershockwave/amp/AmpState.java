package org.dovershockwave.amp;

public record AmpState(String name, double leftPos, double rightPos) {
  public static final AmpState HOME = new AmpState("Home", 0.0, 0.0);
  public static final AmpState AMP = new AmpState("Amp", 19.0, 19.0);
}