package frc.robot.amp;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Amp;
import frc.robot.shuffleboard.TunableSparkPIDController;

public class AmpSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Amp.LEFT_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final SparkPIDController leftPID = leftMotor.getPIDController();

  private final CANSparkMax rightMotor = new CANSparkMax(Amp.RIGHT_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkPIDController rightPID = rightMotor.getPIDController();

  private final boolean manualTuning = false;
  private State desiredState = State.HOME;

  public AmpSubsystem() {
    leftMotor.restoreFactoryDefaults();
    leftMotor.setCANTimeout(250);

    leftMotor.setInverted(true);
    Amp.REV_CONVERSION_FACTOR.apply(leftEncoder);
    leftPID.setP(Amp.LEFT_GAINS.P);
    leftPID.setP(Amp.LEFT_GAINS.I);
    leftPID.setP(Amp.LEFT_GAINS.D);
    leftPID.setOutputRange(Amp.MIN_OUTPUT, Amp.MAX_OUTPUT);
    leftPID.setFeedbackDevice(leftEncoder);
    leftMotor.setSmartCurrentLimit(Constants.Module.TURNING_MOTOR_CURRENT_LIMIT);
    leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    leftMotor.setCANTimeout(0);
    leftMotor.burnFlash();

    rightMotor.restoreFactoryDefaults();
    rightMotor.setCANTimeout(250);

    rightMotor.setInverted(false);
    Amp.REV_CONVERSION_FACTOR.apply(rightEncoder);
    rightPID.setP(Amp.RIGHT_GAINS.P);
    rightPID.setP(Amp.RIGHT_GAINS.I);
    rightPID.setP(Amp.RIGHT_GAINS.D);
    rightPID.setOutputRange(Amp.MIN_OUTPUT, Amp.MAX_OUTPUT);
    rightPID.setFeedbackDevice(rightEncoder);
    rightMotor.setSmartCurrentLimit(Constants.Module.TURNING_MOTOR_CURRENT_LIMIT);
    rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    rightMotor.setCANTimeout(0);
    rightMotor.burnFlash();

    resetPosition();

    final var tab = Shuffleboard.getTab("Amp");
    tab.addNumber("Left Pos", leftEncoder::getPosition);
    tab.add("Left PID", new TunableSparkPIDController(leftPID, () -> desiredState.position, (newPos) -> {
      if (!manualTuning) return;
      this.desiredState = new State("Manual", newPos);
      leftPID.setReference(newPos, CANSparkBase.ControlType.kPosition);
      rightPID.setReference(newPos, CANSparkBase.ControlType.kPosition);
    }));
    tab.addNumber("Right Pos", rightEncoder::getPosition);
    tab.add("Right PID", new TunableSparkPIDController(rightPID));
    tab.addString("State", () -> desiredState.name() + " (" + desiredState.position + "°)");
    tab.add("Reset Position", new InstantCommand(this::resetPosition, this));
    Constants.Tabs.MATCH.addBoolean("Amp At Desired State", this::atDesiredState);
    Constants.Tabs.MATCH.addString("Amp State", () -> desiredState.name() + " (" + desiredState.position + "°)");
  }

  public void setDesiredState(State desiredState) {
    this.desiredState = desiredState;
    leftPID.setReference(desiredState.position, CANSparkBase.ControlType.kPosition);
    rightPID.setReference(desiredState.position, CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return Math.abs(leftEncoder.getPosition() - desiredState.position) < Amp.POSITION_TOLERANCE &&
            Math.abs(rightEncoder.getPosition() - desiredState.position) < Amp.POSITION_TOLERANCE;
  }

  public void resetPosition() {
    leftMotor.setCANTimeout(250);
    leftEncoder.setPosition(0.0);
    leftMotor.setCANTimeout(0);

    rightMotor.setCANTimeout(250);
    rightEncoder.setPosition(0.0);
    rightMotor.setCANTimeout(0);
  }

  /**
   * @param position rev
   */
  public record State(String name, double position) {
    public static final State HOME = new State("Home", 0.0);
    public static final State AMP = new State("Amp", 19.0);
  }
}