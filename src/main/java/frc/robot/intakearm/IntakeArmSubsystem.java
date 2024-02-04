package frc.robot.intakearm;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArm;

public class IntakeArmSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(IntakeArm.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pidController = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private State desiredState;

  public IntakeArmSubsystem() {
    motor.setSmartCurrentLimit(Constants.Module.DRIVING_MOTOR_CURRENT_LIMIT);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    IntakeArm.ANGLE_CONVERSION_FACTOR.apply(encoder, false);

    pidController.setP(IntakeArm.GAINS.P);
    pidController.setI(IntakeArm.GAINS.I);
    pidController.setD(IntakeArm.GAINS.D);
    pidController.setOutputRange(IntakeArm.MIN_OUTPUT, IntakeArm.MAX_OUTPUT);
    pidController.setFeedbackDevice(encoder);

    motor.burnFlash();
  }

  public void setDesiredState(State desiredState) {
    this.desiredState = desiredState;
    pidController.setReference(desiredState.angle, CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return Math.abs(encoder.getPosition() - desiredState.angle) < IntakeArm.ANGLE_TOLERANCE;
  }

  public enum State {
    IN(0.0),
    FLOOR(0.0);

    public final double angle; // degrees

    State(double angle) {
      this.angle = angle;
    }
  }
}