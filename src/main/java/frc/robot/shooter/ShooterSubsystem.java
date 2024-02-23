package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArm;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Tabs;
import frc.robot.shuffleboard.ShuffleboardDouble;
import frc.robot.shuffleboard.TunableSparkPIDController;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private final CANSparkMax leftMotor = new CANSparkMax(Shooter.LEFT_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Shooter.RIGHT_CAN_ID, MotorType.kBrushless);
  private final Servo servo = new Servo(Shooter.SERVO_PWM_ID);

  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final SparkPIDController leftPIDController = leftMotor.getPIDController();
  private final SparkPIDController rightPIDController = rightMotor.getPIDController();

  // private final PIDController leftPIDController = new PIDController(Shooter.GAINS.P, Shooter.GAINS.I, Shooter.GAINS.D);
  // private final PIDController rightPIDController = new PIDController(Shooter.GAINS.P, Shooter.GAINS.I, Shooter.GAINS.D);

  private final ShuffleboardDouble desiredLeftRPS = new ShuffleboardDouble(tab, "Desired Left RPS", 0.0);
  private final ShuffleboardDouble desiredRightRPS = new ShuffleboardDouble(tab, "Desired Right RPS", 0.0);

  private final ShuffleboardDouble flapAngle = new ShuffleboardDouble(tab, "Flap Angle", FlapState.HOME.angle);
  private ShotType type = ShotType.NONE;

//  private final LinearInterpolator RPSInterpolator = new LinearInterpolator(
//
//  );

  // private final VisionSubsystem vision;

  @SuppressWarnings("resource")
  public ShooterSubsystem() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setSmartCurrentLimit(50);
    rightMotor.setSmartCurrentLimit(50);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);

    leftEncoder.setPositionConversionFactor(1);
    leftEncoder.setVelocityConversionFactor((float) 1/60);
    leftPIDController.setP(Shooter.GAINS.P);
    leftPIDController.setP(Shooter.GAINS.I);
    leftPIDController.setP(Shooter.GAINS.D);
    leftPIDController.setOutputRange(-1.0, 1.0);
    leftPIDController.setFeedbackDevice(leftEncoder);

    rightEncoder.setPositionConversionFactor(1);
    rightEncoder.setVelocityConversionFactor((float) 1/60);
    rightPIDController.setP(Shooter.GAINS.P);
    rightPIDController.setP(Shooter.GAINS.I);
    rightPIDController.setP(Shooter.GAINS.D);
    rightPIDController.setFeedbackDevice(rightEncoder);
    rightPIDController.setOutputRange(-1.0, 1.0);
    tab.addNumber("Left RPS", leftEncoder::getVelocity);
    tab.addNumber("Right RPS", rightEncoder::getVelocity);

    leftMotor.burnFlash();
    rightMotor.burnFlash();

    Tabs.MATCH.addNumber("Servo", servo::getAngle)
            .withSize(3, 3)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0.0, "Max", 270.0));

    Tabs.MATCH.addBoolean("At Desired RPS", this::atDesiredRPS);

    tab.add("Left PID", new TunableSparkPIDController(leftPIDController));
    tab.add("Right PID", new TunableSparkPIDController(rightPIDController));
  }

  @Override public void periodic() {
    servo.setAngle(flapAngle.get());
    setRPS(desiredLeftRPS.get());
  }

  private void setRPS(double rps) {
    desiredLeftRPS.set(rps);
    desiredRightRPS.set(rps);

    leftPIDController.setReference(rps, ControlType.kVelocity);
    rightPIDController.setReference(rps, ControlType.kVelocity);
  }

  public boolean atDesiredRPS() {
    return Math.abs(leftEncoder.getVelocity() - type.realRPS) < 5.0 && Math.abs(leftEncoder.getVelocity() - type.realRPS) < 5.0;
  }

  public void rampUp(ShotType type) {
    this.type = type;
    if (type == ShotType.INTERPOLATED) {
//      setRPS(RPSInterpolator.interpolate(vision.getTagRelativeToCenterPose().getX()));
    } else {
      setRPS(type.inputRPS);
    }
  }

  public void stopMotors() {
    desiredLeftRPS.set(0.0);
    desiredRightRPS.set(0.0);

    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setFlapState(FlapState state) {
    flapAngle.set(state.angle);
  }

  public enum ShotType {
    NONE(0.0, 0.0),
    AMP(50.0, 50),
    SUBWOOFER(120.00, 90.0),
    INTERPOLATED(-1.0, -1.0);

    public final double inputRPS;
    public final double realRPS;

    ShotType(double inputRPS, double realRPS) {
      this.inputRPS = inputRPS;
      this.realRPS = realRPS;
    }
  }

  public enum FlapState {
    HOME(0.0),
    SUBWOOFER(125);

    public final double angle;

    FlapState(double angle) {
      this.angle = angle;
    }
  }
}