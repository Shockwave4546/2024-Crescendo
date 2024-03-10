package org.dovershockwave.utils;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public final class SparkUtils {
  private SparkUtils() {
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  public static void runBlockingRel(CANSparkMax spark, TriConsumer<CANSparkMax, RelativeEncoder, SparkPIDController> action) {
    spark.setCANTimeout(250);
    action.accept(spark, spark.getEncoder(), spark.getPIDController());
    spark.setCANTimeout(0);
  }

  public static void configureRel(CANSparkMax spark, TriConsumer<CANSparkMax, RelativeEncoder, SparkPIDController> action) {
    runBlockingRel(spark, (s, t, u) -> {
      action.accept(s, t, u);
      spark.burnFlash();
    });
  }

  public static void runBlockingAbs(CANSparkMax spark, TriConsumer<CANSparkMax, AbsoluteEncoder, SparkPIDController> action) {
    spark.setCANTimeout(250);
    action.accept(spark, spark.getAbsoluteEncoder(), spark.getPIDController());
    spark.setCANTimeout(0);
  }

  public static void configureAbs(CANSparkMax spark, TriConsumer<CANSparkMax, AbsoluteEncoder, SparkPIDController> action) {
    runBlockingAbs(spark, (s, t, u) -> {
      action.accept(s, t, u);
      spark.burnFlash();
    });
  }
}