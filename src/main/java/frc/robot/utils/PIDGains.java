package frc.robot.utils;

public final class PIDGains {
  public final double P;
  public final double I;
  public final double D;

  public PIDGains(double P) {
    this(P, 0.0, 0.0);
  }

  public PIDGains(double P, double I, double D) {
    this.P = P;
    this.I = I;
    this.D = D;
  }
}