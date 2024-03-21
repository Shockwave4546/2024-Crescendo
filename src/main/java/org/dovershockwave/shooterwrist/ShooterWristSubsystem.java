package org.dovershockwave.shooterwrist;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.shuffleboard.TunableSparkPIDController;
import org.dovershockwave.utils.SparkUtils;

import static org.dovershockwave.Constants.*;

public class ShooterWristSubsystem {
  private final CANSparkMax motor = new CANSparkMax(ShooterWrist.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pid = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final boolean manualTuning = true;
  private WristState desiredState = WristState.HOME;

  @SuppressWarnings("resource")
  public ShooterWristSubsystem() {
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
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new WristState("Manual", angle);
      pid.setReference(angle, CANSparkMax.ControlType.kPosition);
    }));
    tab.addString("State", () -> desiredState.name() + " (" + desiredState.angle() + "°)");
    Tabs.MATCH.addBoolean("Wrist At Desired State", this::atDesiredState).withSize(3, 3);
    Tabs.MATCH.addString("Wrist State", () -> desiredState.name() + " (" + desiredState.angle() + "°)").withSize(3, 3);
  }

  public void setDesiredState(WristState desiredState) {
    this.desiredState = desiredState;
    pid.setReference(desiredState.angle(), CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(encoder.getPosition(), desiredState.angle(), ShooterWrist.ANGLE_TOLERANCE);
  }
}