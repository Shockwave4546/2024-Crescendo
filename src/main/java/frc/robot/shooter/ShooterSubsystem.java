package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Tabs;
import frc.robot.shuffleboard.ShuffleboardDouble;

import java.util.Map;

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(Shooter.LEFT_CAN_ID);
  private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(Shooter.RIGHT_CAN_ID);
  private final Servo servo = new Servo(Shooter.SERVO_PWM_ID);

  private final Encoder leftEncoder = new Encoder(Shooter.LEFT_ENCODER[0], Shooter.LEFT_ENCODER[1]);
  private final Encoder rightEncoder = new Encoder(Shooter.RIGHT_ENCODER[0], Shooter.RIGHT_ENCODER[1]);

  private final PIDController leftPIDController = new PIDController(Shooter.GAINS.P, Shooter.GAINS.I, Shooter.GAINS.D, 20);
  private final PIDController rightPIDController = new PIDController(Shooter.GAINS.P, Shooter.GAINS.I, Shooter.GAINS.D, 20);

  private final ShuffleboardDouble desiredLeftRPS = new ShuffleboardDouble(tab, "Desired Left RPS", 0.0);
  private final ShuffleboardDouble desiredRightRPS = new ShuffleboardDouble(tab, "Desired Right RPS", 0.0);

  private final ShuffleboardDouble servoAngle = new ShuffleboardDouble(tab, "Servo Angle", 0.0);

  // private final LinearInterpolator RPMInterpolator = new LinearInterpolator(

  // );

  // private final VisionSubsystem vision;

  @SuppressWarnings("resource")
  public ShooterSubsystem() {
//    leftMotor.configPeakOutputForward(0.85);
//    leftMotor.configPeakOutputReverse(-0.85);
//    rightMotor.configPeakOutputForward(0.85);
//    rightMotor.configPeakOutputReverse(-0.85);
    // this.vision = vision;
    Shooter.REV_CONVERSION_FACTOR.apply(leftEncoder, true);
    Shooter.REV_CONVERSION_FACTOR.apply(rightEncoder, false);
    leftPIDController.setTolerance(40.0, 10.0);
    rightPIDController.setTolerance(40.0, 10.0);

    tab.addNumber("Left RPS", leftEncoder::getRate);
    tab.addNumber("Right RPS", rightEncoder::getRate);

    tab.addNumber("Left RPM", () -> leftEncoder.getRate() * 60.0);
    tab.addNumber("Right RPM", () -> rightEncoder.getRate() * 60.0);

    Tabs.MATCH.addNumber("Servo", servo::getAngle)
            .withSize(3, 3)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0.0, "Max", 270.0));

    Tabs.MATCH.addBoolean("At Desired RPS", this::atDesiredRPS);

    tab.add("Left PID", leftPIDController);
    tab.add("Right PID", rightPIDController);
  }

  @Override public void periodic() {
    servo.setAngle(servoAngle.get());

    leftMotor.set(leftPIDController.calculate(leftEncoder.getRate(), desiredLeftRPS.get()));
    rightMotor.set(rightPIDController.calculate(rightEncoder.getRate(), desiredRightRPS.get()));
  }

  private void setRPM(double rpm) {
    desiredLeftRPS.set(rpm);
    desiredRightRPS.set(rpm);
  }

  public boolean atDesiredRPS() {
    return leftPIDController.atSetpoint() && rightPIDController.atSetpoint();
  }

  public void shootClose() {
    setRPM(Shooter.CLOSE_RPM);
  }

  public void shootInterpolated() {
    // setRPM(RPMInterpolator.interpolate(vision.getTagRelativeToCenterPose().getX()));
  }

  public void stopMotors() {
    desiredLeftRPS.set(0.0);
    desiredRightRPS.set(0.0);

    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void engageServo() {
    servo.setAngle(Shooter.SERVO_SUBWOOFER_ANGLE);
  }

  public void disengageServo() {
    servo.setAngle(Shooter.SERVO_STARTING_ANGLE);
  }

    private final SysIdRoutine rightRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
                  (volt) -> rightMotor.setVoltage(volt.in(Units.Volts)),
                  (log) -> { log.motor("Right Shooter Motor")
                          .angularPosition(Revolutions.of(rightEncoder.get()))
                          .angularVelocity(RevolutionsPerSecond.of(rightEncoder.getRate()))
                          .voltage(Volts.of(rightMotor.getMotorOutputVoltage()));
                    },

                  this)
  );

  public Command runRightQuasiTest(SysIdRoutine.Direction direction) {
    return rightRoutine.quasistatic(direction);
  }

  public Command runRightDynamicTest(SysIdRoutine.Direction direction) {
    return rightRoutine.dynamic(direction);
  }
}