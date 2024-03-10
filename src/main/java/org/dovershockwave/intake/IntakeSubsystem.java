package org.dovershockwave.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.Constants;
import org.dovershockwave.shuffleboard.ShuffleboardBoolean;
import org.dovershockwave.utils.SparkUtils;

public class IntakeSubsystem extends SubsystemBase {
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Intake.LIMIT_SWITCH_DIO_PORT);
  private final CANSparkMax motor = new CANSparkMax(Constants.Intake.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final ShuffleboardBoolean runIdle = new ShuffleboardBoolean(Shuffleboard.getTab("Intake"), "Idle Speed", true)
          .withSize(3, 3);

  @SuppressWarnings("resource")
  public IntakeSubsystem() {
    SparkUtils.configureAbs(motor, (spark, t, u) -> {
      spark.setSmartCurrentLimit(Constants.NeoMotor.NEO_550_CURRENT_LIMIT);
      spark.setIdleMode(CANSparkBase.IdleMode.kBrake);
    });

    Constants.Tabs.MATCH.addBoolean("Has Note", this::hasNote).withSize(3, 3).withPosition(21, 3);

    setDefaultCommand(new IdleIntakeCommand(this));
  }

  public void runIntake(boolean reversed) {
    motor.set(reversed ? Constants.Intake.REVERSE_INTAKE_SPEED : Constants.Intake.FORWARD_INTAKE_SPEED);
  }

  public boolean isRunIdle() {
    return runIdle.get();
  }

  public void runIdle() {
    motor.set(Constants.Intake.IDLE_SPEED);
  }

  public void stopIntake() {
    motor.stopMotor();
  }

  public boolean hasNote() {
    return !limitSwitch.get();
  }
}