package frc.robot.utils;

public class LinearInterpolator {
  private final LinearPair[] pairs;

  /**
   * Note: the x value of the linear pairs must be in ascending order.
   *       ie: [LinearPair(1, 2), LinearPair(2, 3), LinearPair(3, 4)]
   *
   * @param pairs Pairs of (x, y) values
   */
  public LinearInterpolator(LinearPair... pairs) {
    if (pairs.length < 2) {
      throw new IllegalArgumentException("Must have at least 2 pairs");
    }

    this.pairs = pairs;
  }

  public double interpolate(double input) {
    final int length = pairs.length;
    if (input < pairs[0].x) {
      final double slope = calcSlope(pairs[0], pairs[1]);
      return pairs[0].y - (pairs[0].x - input) * slope;
    }

    for (int i = 1; i < length; i++) {
      if (input < pairs[i].x) {
        final double slope = calcSlope(pairs[i - 1], pairs[i]);
        return pairs[i - 1].y + (input - pairs[i - 1].x) * slope;
      }
    }

    final double slope = calcSlope(pairs[length - 2], pairs[length - 1]);
    return pairs[length - 1].y - (input - pairs[length - 1].x) * slope;
  }

  private double calcSlope(LinearPair left, LinearPair right) {
    return (right.y - left.y) / (right.x - left.x);
  }

  public record LinearPair(double x, double y) { }
}