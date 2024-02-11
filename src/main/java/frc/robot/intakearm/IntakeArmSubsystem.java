package frc.robot.intakearm;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArm;
import frc.robot.shuffleboard.ShuffleboardDouble;
import frc.robot.shuffleboard.TunableSparkPIDController;

public class IntakeArmSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake Arm");
  private final CANSparkMax motor = new CANSparkMax(IntakeArm.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pidController = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private State desiredState = State.HOME;

  public IntakeArmSubsystem() {
    motor.restoreFactoryDefaults();

    motor.setSmartCurrentLimit(Constants.Module.DRIVING_MOTOR_CURRENT_LIMIT);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    IntakeArm.ANGLE_CONVERSION_FACTOR.apply(encoder, false);

    encoder.setInverted(true);

    pidController.setP(IntakeArm.GAINS.P);
    pidController.setI(IntakeArm.GAINS.I);
    pidController.setD(IntakeArm.GAINS.D);
    pidController.setOutputRange(IntakeArm.MIN_OUTPUT, IntakeArm.MAX_OUTPUT);
    pidController.setFeedbackDevice(encoder);

    motor.burnFlash();

    tab.addNumber("Duty Cycle", motor::getAppliedOutput);
    tab.addNumber("Curr Angle", encoder::getPosition);
    tab.add("PID Controller", new TunableSparkPIDController(pidController));
    tab.addString("State", () -> desiredState.name());
  }

  @Override public void periodic() {
  }

  public void setDesiredState(State desiredState) {
    this.desiredState = desiredState;
    pidController.setReference(360.0 - desiredState.angle, CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return Math.abs(encoder.getPosition() - desiredState.angle) < IntakeArm.ANGLE_TOLERANCE;
  }

  public boolean atDesiredState(State state) {
    return state == desiredState && atDesiredState();
  }

  public enum State {
    HOME(5.0),
    MIDDLE(50.0),
    FLOOR(180.0);

    public final double angle; // degrees

    State(double angle) {
      this.angle = angle;
    }
  }
}