package org.dovershockwave.shooterwrist;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.dovershockwave.RobotContainer;
import org.dovershockwave.shuffleboard.TunableSparkPIDController;
import org.dovershockwave.utils.LinearInterpolator;
import org.dovershockwave.utils.SparkUtils;

import org.dovershockwave.Constants.NeoMotor;
import org.dovershockwave.Constants.ShooterWrist;
import org.dovershockwave.Constants.Tabs;
import org.dovershockwave.pose.VisionSubsystem;

public class ShooterWristSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(ShooterWrist.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pid = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final boolean manualTuning = true;
  private WristState desiredState = WristState.STARTNG;

  private final LinearInterpolator angleInterpolator = new LinearInterpolator(
    new LinearInterpolator.LinearPair(1.43, 30),
    new LinearInterpolator.LinearPair(2.00, 30),
    new LinearInterpolator.LinearPair(2.65, 38),
    new LinearInterpolator.LinearPair(3.04, 40),
    new LinearInterpolator.LinearPair(4.06, 55)
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
      if (!manualTuning || RobotContainer.isCompetition() || shouldStopWrist()) return;
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

  public void setDesiredState(WristState desiredState) {
    if (shouldStopWrist()) return;
    this.desiredState = desiredState;
    pid.setReference(desiredState.angle(), CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(encoder.getPosition(), desiredState.angle(), ShooterWrist.ANGLE_TOLERANCE);
  }

  public void interpolate() {
    setDesiredState(new WristState("Interpolated", angleInterpolator.interpolate(vision.getCameraToTagTransform().getX())));
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