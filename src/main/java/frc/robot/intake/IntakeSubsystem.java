package frc.robot.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Module;

public class IntakeSubsystem extends SubsystemBase {
  private final DigitalInput limitSwitch = new DigitalInput(Intake.LIMIT_SWITCH_DIO_PORT);
  private final CANSparkMax motor = new CANSparkMax(Intake.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

  public IntakeSubsystem() {
    motor.setSmartCurrentLimit(Module.TURNING_MOTOR_CURRENT_LIMIT);
    motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motor.burnFlash();
  }

  public void runIntake(boolean reversed) {
    motor.set((reversed ? -1.0 : 1.0) * Intake.INTAKE_SPEED);
  }

  public void stopIntake() {
    motor.stopMotor();
  }

  public boolean hasNote() {
    return !limitSwitch.get();
  }
}