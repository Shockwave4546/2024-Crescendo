package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Represents a universal conversion factor for Encoders throughout a SparkMaxRelativeEncoder vs a Quadrature Encoder.
 */
public final class PositionConversionFactor {
  private static final double QUAD_PULSES_PER_REV = 2048.0;
  private final double oneRev;

  /**
   * Represents some default supported types for conversion through the Encoders.
   */
  public static final class ConversionType {
    public static final double DEGREES = 360.0;
    public static final double RADIANS = 2 * Math.PI;
  }

  public PositionConversionFactor(double oneRev) {
    this.oneRev = oneRev;
  }

  /**
   * @param encoder the SparkMaxRelativeEncoder to perform the transformation.
   */
  public void apply(RelativeEncoder encoder, boolean inverted) {
    encoder.setPositionConversionFactor((inverted ? -1.0 : 1.0) * toSparkMaxRelativeEncoder());
  }

    /**
   * @param encoder the SparkMaxAbsoluteEncoder to perform the transformation.
   */
  public void apply(AbsoluteEncoder encoder, boolean inverted) {
    encoder.setPositionConversionFactor((inverted ? -1.0 : 1.0) * toSparkMaxRelativeEncoder());
  }

  /**
   * @param encoder the Encoder to perform the transformation.
   */
  public void apply(Encoder encoder, boolean inverted) {
    encoder.setDistancePerPulse(toQuadEncoder());
    encoder.setReverseDirection(inverted);
  }

  /**
   * Note: a SparkMaxRelativeEncoder has the native units of revolutions, thus the extra division isn't needed.
   *
   * @return the conversion factor for a SparkMaxRelativeEncoder.
   */
  private double toSparkMaxRelativeEncoder() {
    return oneRev;
  }

  /**
   * @return the conversion factor for a Quadrature Encoder.
   */
  private double toQuadEncoder() {
    return oneRev / QUAD_PULSES_PER_REV;
  }
}