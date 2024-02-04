package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Tabs;
import frc.robot.pose.VisionSubsystem;
import frc.robot.shuffleboard.ShuffleboardDouble;
import frc.robot.utils.LinearInterpolator;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(Shooter.LEFT_CAN_ID);
  private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(Shooter.RIGHT_CAN_ID);
  private final Servo servo = new Servo(Shooter.SERVO_PWM_ID);

  private final Encoder leftEncoder = new Encoder(Shooter.LEFT_ENCODER[0], Shooter.LEFT_ENCODER[1]);
  private final Encoder rightEncoder = new Encoder(Shooter.RIGHT_ENCODER[0], Shooter.RIGHT_ENCODER[1]);

  private final PIDController leftPIDController = new PIDController(Shooter.GAINS.P, Shooter.GAINS.I, Shooter.GAINS.D);
  private final PIDController rightPIDController = new PIDController(Shooter.GAINS.P, Shooter.GAINS.I, Shooter.GAINS.D);

  private final ShuffleboardDouble desiredLeftRPM = new ShuffleboardDouble(tab, "Desired Left RPM", 0.0);
  private final ShuffleboardDouble desiredRightRPM = new ShuffleboardDouble(tab, "Desired Right RPM", 0.0);

  private final LinearInterpolator RPMInterpolator = new LinearInterpolator(

  );

  private final VisionSubsystem vision;

  @SuppressWarnings("resource")
  public ShooterSubsystem(VisionSubsystem vision) {
    this.vision = vision;
    Shooter.REV_CONVERSION_FACTOR.apply(leftEncoder, false);
    Shooter.REV_CONVERSION_FACTOR.apply(rightEncoder, false);
    leftPIDController.setTolerance(2.0, 2.0);
    rightPIDController.setTolerance(2.0, 2.0);

    tab.addNumber("Left RPM", () -> leftEncoder.getRate() * 60.0);
    tab.addNumber("Right RPM", () -> rightEncoder.getRate() * 60.0);
    Tabs.MATCH.addNumber("Servo", servo::getAngle)
            .withSize(3, 3)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0.0, "Max", 270.0));

    Tabs.MATCH.addBoolean("At Desired RPM", this::atDesiredRPM);
  }

  @Override public void periodic() {
    leftMotor.set(leftPIDController.calculate(leftEncoder.getRate() * 60.0, desiredLeftRPM.get()));
    rightMotor.set(rightPIDController.calculate(rightEncoder.getRate() * 60.0, desiredRightRPM.get()));
  }

  private void setRPM(double rpm) {
    desiredLeftRPM.set(rpm);
    desiredRightRPM.set(rpm);
  }

  public boolean atDesiredRPM() {
    return leftPIDController.atSetpoint() && rightPIDController.atSetpoint();
  }

  public void shootClose() {
    setRPM(Shooter.CLOSE_RPM);
  }

  public void shootInterpolated() {
    setRPM(RPMInterpolator.interpolate(vision.getTagRelativeToCenterPose().getX()));
  }

  public void stopMotors() {
    desiredLeftRPM.set(0.0);
    desiredRightRPM.set(0.0);

    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void engageServo() {
    servo.setAngle(Shooter.SERVO_SUBWOOFER_ANGLE);
  }

  public void disengageServo() {
    servo.setAngle(Shooter.SERVO_STARTING_ANGLE);
  }
}