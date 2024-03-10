package org.dovershockwave.utils;

/**
 * A class to store PID gains. The types are [float] because the Rev API uses float32 for PID gains.
 * <a href="https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters#:~:text=of%20the%20controller.-,kP_1,-21">...</a>
 */
public record PIDGains(float P, float I, float D, float FF) {
  public PIDGains(float P, float I, float D) {
    this(P, I, D, 0.0F);
  }

  public PIDGains(float P) {
    this(P, 0.0F, 0.0F, 0.0F);
  }
}