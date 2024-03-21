package org.dovershockwave.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.shuffleboard.TunableSparkPIDController;
import org.dovershockwave.utils.LinearInterpolator;
import org.dovershockwave.utils.SparkUtils;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax bottomMotor = new CANSparkMax(Constants.Shooter.BOTTOM_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax topMotor = new CANSparkMax(Constants.Shooter.TOP_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private final RelativeEncoder topEncoder = topMotor.getEncoder();

  private final SparkPIDController bottomPID = bottomMotor.getPIDController();
  private final SparkPIDController topPID = topMotor.getPIDController();

  private final boolean manualTuning = true;
  private ShooterState desiredState = ShooterState.STOPPED;

  private final LinearInterpolator RPSInterpolator = new LinearInterpolator(
    new LinearInterpolator.LinearPair(2.5, 28.0),
    new LinearInterpolator.LinearPair(2.3, 28.0),
    new LinearInterpolator.LinearPair(2.1, 29.0),
    new LinearInterpolator.LinearPair(2.0, 33.0),
    new LinearInterpolator.LinearPair(1.6, 45.0),
    new LinearInterpolator.LinearPair(1.55, 50.0)
  );

  private final VisionSubsystem vision;

  @SuppressWarnings("resource")
  public ShooterSubsystem(VisionSubsystem vision) {
    this.vision = vision;

    final var tab = Shuffleboard.getTab("Shooter");
    SparkUtils.configureRel(bottomMotor, (motor, encoder, pid) -> {
      motor.setSmartCurrentLimit(Constants.NeoMotor.NEO_CURRENT_LIMIT);
      motor.setInverted(Constants.Shooter.LEFT_INVERTED);
      motor.setIdleMode(IdleMode.kCoast);
      Constants.Shooter.REV_CONVERSION_FACTOR.apply(encoder);
      encoder.setVelocityConversionFactor(Constants.Shooter.RPS_CONVERSION_FACTOR);
      pid.setP(Constants.Shooter.GAINS.P());
      pid.setI(Constants.Shooter.GAINS.I());
      pid.setD(Constants.Shooter.GAINS.D());
      pid.setFF(Constants.Shooter.GAINS.FF());
      pid.setOutputRange(Constants.Shooter.MIN_OUTPUT, Constants.Shooter.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
    });
    tab.addNumber("Bottom RPS", bottomEncoder::getVelocity);
    tab.add("Bottom PID", new TunableSparkPIDController(bottomPID, () -> desiredState.bottomRPS(), (bottomRPS) -> {
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new ShooterState("Manual", bottomRPS, desiredState.topRPS());
      bottomPID.setReference(bottomRPS, ControlType.kVelocity);
    }));

    SparkUtils.configureRel(topMotor, (motor, encoder, pid) -> {
      motor.setSmartCurrentLimit(Constants.NeoMotor.NEO_CURRENT_LIMIT);
      motor.setInverted(Constants.Shooter.RIGHT_INVERTED);
      motor.setIdleMode(IdleMode.kCoast);
      Constants.Shooter.REV_CONVERSION_FACTOR.apply(encoder);
      encoder.setVelocityConversionFactor(Constants.Shooter.RPS_CONVERSION_FACTOR);
      pid.setP(Constants.Shooter.GAINS.P());
      pid.setI(Constants.Shooter.GAINS.I());
      pid.setD(Constants.Shooter.GAINS.D());
      pid.setFF(Constants.Shooter.GAINS.FF());
      pid.setFeedbackDevice(encoder);
      pid.setOutputRange(Constants.Shooter.MIN_OUTPUT, Constants.Shooter.MAX_OUTPUT);
    });
    tab.addNumber("Top RPS", topEncoder::getVelocity);
    tab.add("Top PID", new TunableSparkPIDController(topPID, () -> desiredState.topRPS(), (topRPS) -> {
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new ShooterState("Manual", desiredState.bottomRPS(), topRPS);
      bottomPID.setReference(topRPS, ControlType.kVelocity);
    }));

    Constants.Tabs.MATCH.addBoolean("Shooter At Desired State", this::atDesiredRPS).withSize(3, 3).withPosition(24, 3);
    Constants.Tabs.MATCH.addString("Shooter State", () -> desiredState.name() + " (" + desiredState.bottomRPS() + ", " +  desiredState.topRPS() + ")").withSize(3, 3).withPosition(24, 6);
  }

  private void setDesiredState(ShooterState state) {
    this.desiredState = state;
    bottomPID.setReference(state.bottomRPS(), ControlType.kVelocity);
    topPID.setReference(state.topRPS(), ControlType.kVelocity);
  }

  public boolean atDesiredRPS() {
    return MathUtil.isNear(bottomEncoder.getVelocity(), desiredState.bottomRPS(), Constants.Shooter.RPS_TOLERANCE) && MathUtil.isNear(topEncoder.getVelocity(), desiredState.topRPS(), Constants.Shooter.RPS_TOLERANCE);
  }

  public void rampUp(ShooterState type) {
    if (type == ShooterState.INTERPOLATED) {
      if (!vision.hasViableTarget()) return;
      final var distance = vision.getCameraToTagTransform().getX();
      if (distance <= 1.5) {
        setDesiredState(ShooterState.SUBWOOFER);
        return;
      }

      final var rps = RPSInterpolator.interpolate(distance);
      setDesiredState(new ShooterState("Interpolated", rps, rps));
    } else {
      setDesiredState(type);
    }
  }

  public void stopMotors() {
    this.desiredState = ShooterState.STOPPED;
    bottomMotor.stopMotor();
    topMotor.stopMotor();
  }
}