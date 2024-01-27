package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(ShooterConstants.LEFT_CAN_ID);
  private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(ShooterConstants.RIGHT_CAN_ID);
  private final Servo servo = new Servo(ShooterConstants.SERVO_PWM_ID);
}