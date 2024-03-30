package org.dovershockwave.shooterwrist;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.Constants.NeoMotor;
import org.dovershockwave.Constants.ShooterWrist;
import org.dovershockwave.Constants.Tabs;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.shuffleboard.TunableSparkPIDController;
import org.dovershockwave.utils.LinearInterpolator;
import org.dovershockwave.utils.SparkUtils;

import static org.dovershockwave.Constants.Debug;

import org.dovershockwave.Constants.Debug;

public class ShooterWristSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(ShooterWrist.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pid = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private WristState desiredState = WristState.STARTING;

  private final LinearInterpolator angleInterpolator = new LinearInterpolator(
    new LinearInterpolator.LinearPair(1.40, 30),
    new LinearInterpolator.LinearPair(1.60, 30),
    new LinearInterpolator.LinearPair(1.80, 32),
    new LinearInterpolator.LinearPair(2.00, 32),
    new LinearInterpolator.LinearPair(2.20, 34),
    new LinearInterpolator.LinearPair(2.40, 40),
    new LinearInterpolator.LinearPair(2.60, 42),
    new LinearInterpolator.LinearPair(2.70, 45),
    new LinearInterpolator.LinearPair(2.80, 47),
    new LinearInterpolator.LinearPair(3.00, 47),
    new LinearInterpolator.LinearPair(3.20, 47),
    new LinearInterpolator.LinearPair(3.40, 47),
    new LinearInterpolator.LinearPair(4.00, 47)
  );

  private final VisionSubsystem vision;

  @SuppressWarnings("resource")
  public ShooterWristSubsystem(VisionSubsystem vision) {
    this.vision = vision;
    SparkUtils.configureAbs(motor, (motor, encoder, pid) -> {
      motor.setSmartCurrentLimit(NeoMotor.NEO_CURRENT_LIMIT);
      motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      ShooterWrist.ANGLE_CONVERSION_FACTOR.apply(encoder);
      encoder.setInverted(ShooterWrist.INVERTED);
      encoder.setZeroOffset(ShooterWrist.ANGLE_OFFSET);

      pid.setP(ShooterWrist.GAINS.P());
      pid.setI(ShooterWrist.GAINS.I());
      pid.setD(ShooterWrist.GAINS.D());
      pid.setOutputRange(ShooterWrist.MIN_OUTPUT, ShooterWrist.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
    });

    final var tab = Shuffleboard.getTab("Shooter Wrist");
    tab.addNumber("Duty Cycle", motor::getAppliedOutput);
    tab.addNumber("Current Angle", encoder::getPosition);
    tab.add("PID", new TunableSparkPIDController(pid, () -> desiredState.angle(), (angle) -> {
      if (!Debug.MANUAL_TUNING || RobotContainer.isCompetition() || shouldStopWrist()) return;
      final var clamped = clampAngle(angle);
      this.desiredState = new WristState("Manual", clamped);
      pid.setReference(clamped, CANSparkMax.ControlType.kPosition);
    }));
    tab.addString("State", () -> desiredState.name() + " (" + desiredState.angle() + "°)");
    Tabs.MATCH.addBoolean("Wrist At Desired State", this::atDesiredState).withSize(3, 3);
    Tabs.MATCH.addString("Wrist State", () -> desiredState.name() + " (" + desiredState.angle() + "°)").withSize(3, 3);
  }

  @Override public void periodic() {
    if (shouldStopWrist()) {
      DriverStation.reportError("The encoder is reporting an angle that will break the wrist: " + encoder.getPosition(), false);
    }
  }
  
  public double clampAngle(double angle) {
    return MathUtil.clamp(angle, ShooterWrist.MIN_ANGLE, ShooterWrist.MAX_ANGLE);
  }

  public void setDesiredState(WristState state) {
    if (shouldStopWrist()) return;
    if (state == WristState.INTERPOLATED) {
      if (!vision.hasViableTarget()) return;
      final var transform = vision.getCameraToTagTransform(RobotContainer.getSubwooferTagID());
      if (transform == null) return;
      final var distance = transform.getX();
      if (distance > 3.90) return;
      this.desiredState = new WristState("Interpolated", angleInterpolator.interpolate(distance));
    } else {
      this.desiredState = state;
    }

    pid.setReference(clampAngle(desiredState.angle()), CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(encoder.getPosition(), desiredState.angle(), ShooterWrist.ANGLE_TOLERANCE);
  }

  /**
   * If the Encoder is reading an angle that causes the wrist to go into the robot, it should stop.
   * These angles include [81, 360].
   *
   * @return whether the wrist should stop operating as to not break it.
   */
  private boolean shouldStopWrist() {
    return encoder.getPosition() > ShooterWrist.MAX_ANGLE;
  }
}