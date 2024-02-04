package frc.robot.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Module;
import frc.robot.Constants.Tabs;

public class IntakeSubsystem extends SubsystemBase {
  private final DigitalInput leftSwitch = new DigitalInput(Intake.LEFT_SWITCH_DIO_PORT);
  private final DigitalInput rightSwitch = new DigitalInput(Intake.RIGHT_SWITCH_DIO_PORT);
  private final CANSparkMax motor = new CANSparkMax(Intake.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

  @SuppressWarnings("resource")
  public IntakeSubsystem() {
    motor.setSmartCurrentLimit(Module.TURNING_MOTOR_CURRENT_LIMIT);
    motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motor.burnFlash();

    Tabs.MATCH.addBoolean("Has Note", this::hasNote);
  }

  public void runIntake(boolean reversed) {
    motor.set(reversed ? Intake.REVERSE_INTAKE_SPEED : Intake.FORWARD_INTAKE_SPEED);
  }

  public void stopIntake() {
    motor.stopMotor();
  }

  public boolean hasNote() {
    return leftSwitch.get() && rightSwitch.get();
  }
}