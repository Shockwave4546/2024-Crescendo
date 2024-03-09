package frc.robot.utils;

/**
 * A class to store PID gains. The types are [float] because the Rev API uses float32 for PID gains.
 * <a href="https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters#:~:text=of%20the%20controller.-,kP_1,-21">...</a>
 */
public final class PIDGains {
  public final float P;
  public final float I;
  public final float D;

  public PIDGains(float P) {
    this(P, 0.0F, 0.0F);
  }

  public PIDGains(float P, float I, float D) {
    this.P = P;
    this.I = I;
    this.D = D;
  }
}