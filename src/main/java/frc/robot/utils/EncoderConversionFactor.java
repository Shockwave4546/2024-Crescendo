package frc.robot.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Represents a universal conversion factor for Encoders throughout a SparkMaxRelativeEncoder vs a Quadrature Encoder.
 */
public final class EncoderConversionFactor {
  private static final double QUAD_PULSES_PER_REV = 2048.0;
  private final double gearRatio;
  private final double oneRev;

  /**
   * Represents some default supported types for conversion through the Encoders.
   */
  public static final class ConversionType {
    public static final double DEGREES = 360.0;
    public static final double RADIANS = 2 * Math.PI;
    public static final double SIX_IN_WHEELS_POS = Units.inchesToMeters(6) * Math.PI;
  }

  public EncoderConversionFactor(double oneRev, double gearRatio) {
    this.oneRev = oneRev;
    this.gearRatio = gearRatio;
  }

  public EncoderConversionFactor(double oneRev) {
    this(oneRev, 1.0);
  }

  /**
   * @param encoder the SparkMaxRelativeEncoder to perform the transformation.
   */
  public void applyTo(RelativeEncoder encoder, boolean inverted) {
    encoder.setPositionConversionFactor((inverted ? -1.0 : 1.0) * toSparkMaxRelativeEncoder());
  }

    /**
   * @param encoder the SparkMaxAbsoluteEncoder to perform the transformation.
   */
  public void applyTo(AbsoluteEncoder encoder, boolean inverted) {
    encoder.setPositionConversionFactor((inverted ? -1.0 : 1.0) * toSparkMaxRelativeEncoder());
  }

  /**
   * @param encoder the Encoder to perform the transformation.
   */
  public void applyTo(Encoder encoder, boolean inverted) {
    encoder.setDistancePerPulse(toQuadEncoder());
    encoder.setReverseDirection(inverted);
  }

  /**
   * Note: a SparkMaxRelativeEncoder has the native units of revolutions, thus the extra division isn't needed.
   *
   * @return the conversion factor for a SparkMaxRelativeEncoder.
   */
  private double toSparkMaxRelativeEncoder() {
    return oneRev * gearRatio;
  }

  /**
   * @return the conversion factor for a Quadrature Encoder.
   */
  private double toQuadEncoder() {
    return (oneRev * gearRatio) / QUAD_PULSES_PER_REV;
  }
}