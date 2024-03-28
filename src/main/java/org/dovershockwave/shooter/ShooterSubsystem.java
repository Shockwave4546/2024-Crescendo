package org.dovershockwave.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.shuffleboard.ShuffleboardBoolean;
import org.dovershockwave.shuffleboard.TunableSparkPIDController;
import org.dovershockwave.utils.LinearInterpolator;
import org.dovershockwave.utils.SparkUtils;

import static org.dovershockwave.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax bottomMotor = new CANSparkMax(Shooter.BOTTOM_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax topMotor = new CANSparkMax(Shooter.TOP_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private final RelativeEncoder topEncoder = topMotor.getEncoder();

  private final SparkPIDController bottomPID = bottomMotor.getPIDController();
  private final SparkPIDController topPID = topMotor.getPIDController();

  private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private final ShuffleboardBoolean runIdle = new ShuffleboardBoolean(tab, "Idle Speed", true).withSize(3, 3);

  private ShooterState desiredState = ShooterState.STOPPED;

  private final LinearInterpolator bottomRPSInterpolator = new LinearInterpolator(
    new LinearInterpolator.LinearPair(1.40, 60),
    new LinearInterpolator.LinearPair(1.60, 60),
    new LinearInterpolator.LinearPair(1.80, 60),
    new LinearInterpolator.LinearPair(2.00, 50),
    new LinearInterpolator.LinearPair(2.20, 50),
    new LinearInterpolator.LinearPair(2.40, 50),
    new LinearInterpolator.LinearPair(2.60, 50),
    new LinearInterpolator.LinearPair(2.80, 45),
    new LinearInterpolator.LinearPair(3.00, 50),
    new LinearInterpolator.LinearPair(3.20, 45),
    new LinearInterpolator.LinearPair(3.40, 45),
    new LinearInterpolator.LinearPair(3.40, 50)
  );

  private final LinearInterpolator topRPSInterpolator = new LinearInterpolator(
    new LinearInterpolator.LinearPair(1.40, 35),
    new LinearInterpolator.LinearPair(1.60, 35),
    new LinearInterpolator.LinearPair(1.80, 40),
    new LinearInterpolator.LinearPair(2.00, 55),
    new LinearInterpolator.LinearPair(2.20, 60),
    new LinearInterpolator.LinearPair(2.40, 65),
    new LinearInterpolator.LinearPair(2.60, 65),
    new LinearInterpolator.LinearPair(2.80, 75),
    new LinearInterpolator.LinearPair(3.00, 80),
    new LinearInterpolator.LinearPair(3.20, 70),
    new LinearInterpolator.LinearPair(3.40, 65),
    new LinearInterpolator.LinearPair(4.00, 70)
  );

  private final VisionSubsystem vision;

  @SuppressWarnings("resource")
  public ShooterSubsystem(VisionSubsystem vision) {
    this.vision = vision;

    SparkUtils.configureRel(bottomMotor, (motor, encoder, pid) -> {
      motor.setSmartCurrentLimit(NeoMotor.NEO_CURRENT_LIMIT);
      motor.setInverted(Shooter.LEFT_INVERTED);
      motor.setIdleMode(IdleMode.kCoast);
      Shooter.REV_CONVERSION_FACTOR.apply(encoder);
      encoder.setVelocityConversionFactor(Shooter.RPS_CONVERSION_FACTOR);
      pid.setP(Shooter.GAINS.P());
      pid.setI(Shooter.GAINS.I());
      pid.setD(Shooter.GAINS.D());
      pid.setFF(Shooter.GAINS.FF());
      pid.setOutputRange(Shooter.MIN_OUTPUT, Shooter.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
    });
    tab.addNumber("Bottom RPS", bottomEncoder::getVelocity);
    tab.add("Bottom PID", new TunableSparkPIDController(bottomPID, () -> desiredState.bottomRPS(), (bottomRPS) -> {
      if (!Debug.MANUAL_TUNING || RobotContainer.isCompetition()) return;
      this.desiredState = new ShooterState("Manual", bottomRPS, desiredState.topRPS());
      bottomPID.setReference(bottomRPS, ControlType.kVelocity);
    }));

    SparkUtils.configureRel(topMotor, (motor, encoder, pid) -> {
      motor.setSmartCurrentLimit(NeoMotor.NEO_CURRENT_LIMIT);
      motor.setInverted(Shooter.RIGHT_INVERTED);
      motor.setIdleMode(IdleMode.kCoast);
      Shooter.REV_CONVERSION_FACTOR.apply(encoder);
      encoder.setVelocityConversionFactor(Shooter.RPS_CONVERSION_FACTOR);
      pid.setP(Shooter.GAINS.P());
      pid.setI(Shooter.GAINS.I());
      pid.setD(Shooter.GAINS.D());
      pid.setFF(Shooter.GAINS.FF());
      pid.setFeedbackDevice(encoder);
      pid.setOutputRange(Shooter.MIN_OUTPUT, Shooter.MAX_OUTPUT);
    });
    tab.addNumber("Top RPS", topEncoder::getVelocity);
    tab.add("Top PID", new TunableSparkPIDController(topPID, () -> desiredState.topRPS(), (topRPS) -> {
      if (!Debug.MANUAL_TUNING || RobotContainer.isCompetition()) return;
      this.desiredState = new ShooterState("Manual", desiredState.bottomRPS(), topRPS);
      topPID.setReference(topRPS, ControlType.kVelocity);
    }));

    Tabs.MATCH.addBoolean("Shooter At Desired State", this::atDesiredState).withSize(3, 3).withPosition(24, 3);
    Tabs.MATCH.addString("Shooter State", () -> desiredState.name() + " (" + desiredState.bottomRPS() + ", " +  desiredState.topRPS() + ")").withSize(3, 3).withPosition(24, 6);
  }

  public void setDesiredState(ShooterState state) {
    if (state == ShooterState.INTERPOLATED) {
      if (!vision.hasViableTarget()) return;
      final var transform = vision.getCameraToTagTransform(RobotContainer.getSubwooferTagID());
      if (transform == null) return;
      final var distance = transform.getX();
      this.desiredState = new ShooterState("Interpolated", bottomRPSInterpolator.interpolate(distance), topRPSInterpolator.interpolate(distance));
    } else {
      this.desiredState = state;
    }

    bottomPID.setReference(desiredState.bottomRPS(), ControlType.kVelocity);
    topPID.setReference(desiredState.topRPS(), ControlType.kVelocity);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(bottomEncoder.getVelocity(), desiredState.bottomRPS(), Shooter.RPS_TOLERANCE) && MathUtil.isNear(topEncoder.getVelocity(), desiredState.topRPS(), Shooter.RPS_TOLERANCE);
  }

  public boolean isRunIdle() {
    return runIdle.get();
  }

  public void stopMotors() {
    this.desiredState = ShooterState.STOPPED;
    bottomMotor.stopMotor();
    topMotor.stopMotor();
  }
}