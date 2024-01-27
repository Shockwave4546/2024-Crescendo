package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final DigitalInput limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_DIO_PORT);
  private final CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
}
