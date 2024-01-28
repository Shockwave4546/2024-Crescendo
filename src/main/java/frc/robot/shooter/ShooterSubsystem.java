package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Tabs;
import frc.robot.shuffleboard.ShuffleboardDouble;

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

  private final ShuffleboardDouble desiredLeftVelocity = new ShuffleboardDouble(tab, "Desired Left Velocity", 0.0);
  private final ShuffleboardDouble desiredRightVelocity = new ShuffleboardDouble(tab, "Desired Right Velocity", 0.0);

  public ShooterSubsystem() {
    Shooter.MPS_CONVERSION_FACTOR.applyTo(leftEncoder, false);
    Shooter.MPS_CONVERSION_FACTOR.applyTo(rightEncoder, false);
    leftPIDController.setTolerance(2.0, 2.0);
    rightPIDController.setTolerance(2.0, 2.0);

    tab.add("Left Velocity", leftEncoder);
    tab.add("Right Velocity", rightEncoder);
    Tabs.MATCH.addNumber("Servo", servo::getAngle)
            .withSize(3, 3)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0.0, "Max", 270.0));

    Tabs.MATCH.addBoolean("Shooter At Speed", () -> leftPIDController.atSetpoint() && rightPIDController.atSetpoint());
  }

  @Override public void periodic() {
    leftMotor.set(leftPIDController.calculate(leftEncoder.getRate(), desiredLeftVelocity.get()));
    desiredRightVelocity.set(rightPIDController.calculate(rightEncoder.getRate(), rightEncoder.get()));
  }

  public void setVelocity(double velocity) {
    desiredLeftVelocity.set(velocity);
    desiredRightVelocity.set(velocity);
  }

  public void stopMotors() {
    desiredLeftVelocity.set(0.0);
    desiredRightVelocity.set(0.0);

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