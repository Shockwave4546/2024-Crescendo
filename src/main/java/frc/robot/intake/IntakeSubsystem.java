package frc.robot.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Module;
import frc.robot.Constants.Tabs;
import frc.robot.shuffleboard.ShuffleboardBoolean;

public class IntakeSubsystem extends SubsystemBase {
  private final DigitalInput limitSwitch = new DigitalInput(Intake.LIMIT_SWITCH_DIO_PORT);
  private final CANSparkMax motor = new CANSparkMax(Intake.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final ShuffleboardBoolean runIdle = new ShuffleboardBoolean(Shuffleboard.getTab("Intake"), "Idle Speed", true);

  @SuppressWarnings("resource")
  public IntakeSubsystem() {
    motor.restoreFactoryDefaults();

    motor.setSmartCurrentLimit(Module.TURNING_MOTOR_CURRENT_LIMIT);
    motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motor.burnFlash();

    Tabs.MATCH.addBoolean("Has Note", this::hasNote);

     setDefaultCommand(new IdleIntakeCommand(this));
  }

  public void runIntake(boolean reversed) {
    motor.set(reversed ? Intake.REVERSE_INTAKE_SPEED : Intake.FORWARD_INTAKE_SPEED);
  }

  public boolean isRunIdle() {
    return runIdle.get();
  }

  public void runIdle() {
    motor.set(-0.2);
  }

  public void stopIntake() {
    motor.stopMotor();
  }

  public boolean hasNote() {
    return !limitSwitch.get();
  }
}