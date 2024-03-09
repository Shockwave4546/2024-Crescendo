package frc.robot.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Tabs;
import frc.robot.pose.VisionSubsystem;
import frc.robot.shuffleboard.ShuffleboardDouble;
import frc.robot.shuffleboard.TunableSparkPIDController;
import frc.robot.utils.LinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private final CANSparkMax bottomMotor = new CANSparkMax(Shooter.BOTTOM_CAN_ID, MotorType.kBrushless); 
  private final CANSparkMax topMotor = new CANSparkMax(Shooter.TOP_CAN_ID, MotorType.kBrushless); 

  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private final RelativeEncoder topEncoder = topMotor.getEncoder();

  private final SparkPIDController bottomPID = bottomMotor.getPIDController();
  private final SparkPIDController topPID = topMotor.getPIDController();

  private final ShuffleboardDouble desiredBottomRPS = new ShuffleboardDouble(tab, "Desired Bottom RPS", 0.0);
  private final ShuffleboardDouble desiredTopRPS = new ShuffleboardDouble(tab, "Desired Top RPS", 0.0);

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

    bottomMotor.restoreFactoryDefaults();
    bottomMotor.setCANTimeout(250);

    bottomMotor.setSmartCurrentLimit(50);
    bottomMotor.setInverted(true);
    bottomMotor.setIdleMode(IdleMode.kCoast);
    Shooter.REV_CONVERSION_FACTOR.apply(bottomEncoder);
    bottomEncoder.setVelocityConversionFactor(Shooter.RPS_CONVERSION_FACTOR);
    bottomPID.setP((float) Shooter.GAINS.P);
    bottomPID.setP(Shooter.GAINS.I);
    bottomPID.setP(Shooter.GAINS.D);
    bottomPID.setFF(Shooter.FF);
    bottomPID.setOutputRange(Shooter.MIN_OUTPUT, Shooter.MAX_OUTPUT);
    bottomPID.setFeedbackDevice(bottomEncoder);

    bottomMotor.setCANTimeout(0);
    bottomMotor.burnFlash();

    topMotor.restoreFactoryDefaults();
    topMotor.setCANTimeout(250);

    topMotor.setSmartCurrentLimit(50);
    topMotor.setInverted(true);
    topMotor.setIdleMode(IdleMode.kCoast);
    Shooter.REV_CONVERSION_FACTOR.apply(topEncoder);
    topEncoder.setVelocityConversionFactor(Shooter.RPS_CONVERSION_FACTOR);
    topPID.setP((float) Shooter.GAINS.P);
    topPID.setP(Shooter.GAINS.I);
    topPID.setP(Shooter.GAINS.D);
    topPID.setFF(Shooter.FF);
    topPID.setFeedbackDevice(topEncoder);
    topPID.setOutputRange(Shooter.MIN_OUTPUT, Shooter.MAX_OUTPUT);

    topMotor.setCANTimeout(0);
    topMotor.burnFlash();

    Tabs.MATCH.addBoolean("At Desired RPS", this::atDesiredRPS);

    tab.addNumber("Bottom RPS", bottomEncoder::getVelocity);
    tab.addNumber("Top RPS", topEncoder::getVelocity);
    tab.add("Left PID", new TunableSparkPIDController(bottomPID));
    tab.add("Right PID", new TunableSparkPIDController(topPID));
  }

  @Override public void periodic() {
    setRPS(desiredBottomRPS.get(), desiredTopRPS.get());
  }

  private void setRPS(double bottomRPS, double topRPS) {
    desiredBottomRPS.set(bottomRPS);
    desiredTopRPS.set(topRPS);

    bottomPID.setReference(bottomRPS, ControlType.kVelocity);
    topPID.setReference(topRPS, ControlType.kVelocity);
  }

  public boolean atDesiredRPS() {
    return (Math.abs(bottomEncoder.getVelocity() - desiredBottomRPS.get()) < Shooter.RPS_TOLERANCE) && (Math.abs(topEncoder.getVelocity() - desiredTopRPS.get()) < Shooter.RPS_TOLERANCE);
  }

  public void rampUp(ShotType type) {
    if (type == ShotType.INTERPOLATED) {
      if (!vision.hasViableTarget()) return;
      final var distance = vision.getCameraToTagTransform().getX();
      if (distance <= 1.5) {
        desiredBottomRPS.set(70.0);
        desiredTopRPS.set(30.0);
        setRPS(70.0, 30.0);
        return;
      }

      final var rps = RPSInterpolator.interpolate(distance);
      desiredBottomRPS.set(rps);
      desiredTopRPS.set(rps);
      setRPS(rps, rps);
    } else {
      desiredBottomRPS.set(type.bottomRPS);
      desiredTopRPS.set(type.topRPS);
      setRPS(type.bottomRPS, type.topRPS);
    }
  }

  public void stopMotors() {
    desiredBottomRPS.set(0.0);
    desiredTopRPS.set(0.0);

    bottomMotor.stopMotor();
    topMotor.stopMotor();
  }

  public enum ShotType {
    NONE(0.0, 0.0),
    AMP(30.0, 30.0),
    SUBWOOFER(70.0, 30.0),
    FAR(50.0, 50.0),
    INTERPOLATED(-1.0, -1.0);

    public final double bottomRPS;
    public final double topRPS;

    ShotType(double bottomRPS, double topRPS) {
      this.bottomRPS = bottomRPS;
      this.topRPS = topRPS;
    }
  }
}