package frc.robot.intakearm;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArm;
import frc.robot.Constants.Tabs;
import frc.robot.shuffleboard.TunableSparkPIDController;

public class IntakeArmSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(IntakeArm.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pidController = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private State desiredState = State.HOME;

  @SuppressWarnings("resource")
  public IntakeArmSubsystem() {
    motor.restoreFactoryDefaults();

    motor.setSmartCurrentLimit(Constants.Module.DRIVING_MOTOR_CURRENT_LIMIT);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    IntakeArm.ANGLE_CONVERSION_FACTOR.apply(encoder);
    encoder.setInverted(false);
    encoder.setZeroOffset(IntakeArm.ANGLE_OFFSET);

    pidController.setP(IntakeArm.GAINS.P);
    pidController.setI(IntakeArm.GAINS.I);
    pidController.setD(IntakeArm.GAINS.D);
    pidController.setOutputRange(IntakeArm.MIN_OUTPUT, IntakeArm.MAX_OUTPUT);
    pidController.setFeedbackDevice(encoder);

    motor.burnFlash();

    final var tab = Shuffleboard.getTab("Intake Arm");
    tab.addNumber("Duty Cycle", motor::getAppliedOutput);
    tab.addNumber("Current Angle", encoder::getPosition);
    tab.add("PID Controller", new TunableSparkPIDController(pidController));
    tab.addString("State", () -> desiredState.name() + " (" + desiredState.angle + "°)");
    Tabs.MATCH.addBoolean("Arm At Desired State", this::atDesiredState);
    Tabs.MATCH.addString("Arm State", () -> desiredState.name() + " (" + desiredState.angle + "°)");
    Tabs.MATCH.addBoolean("Arm Valid Encoder", () -> !shouldStopArm());
  }

  @Override public void periodic() {
     if (shouldStopArm()) {
       DriverStation.reportError("The encoder is reporting an angle that will break the arm: " + encoder.getPosition(), false);
     }
  }

  public void setDesiredState(State desiredState) {
    if (shouldStopArm()) return;
    this.desiredState = desiredState;
    pidController.setReference(desiredState.angle, CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return Math.abs(encoder.getPosition() - desiredState.angle) < IntakeArm.ANGLE_TOLERANCE;
  }

  /**
   * If the Encoder is reading an angle that causes the arm to go into the robot, it should stop.
   * These angles include [195, 360].
   *
   * @return whether the arm should stop operating as to not break it.
   */
  private boolean shouldStopArm() {
    return encoder.getPosition() > 198.0;
  }

  public enum State {
    HOME(5.0),
    MIDDLE(50.0),
    FLOOR(195.0);

    public final double angle; // degrees

    State(double angle) {
      this.angle = angle;
    }
  }
}