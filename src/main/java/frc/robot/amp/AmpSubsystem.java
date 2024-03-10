package frc.robot.amp;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Amp;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.TunableSparkPIDController;
import frc.robot.utils.SparkUtils;

import static frc.robot.Constants.NeoMotor;
import static frc.robot.Constants.Tabs;

public class AmpSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Amp.LEFT_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final SparkPIDController leftPID = leftMotor.getPIDController();

  private final CANSparkMax rightMotor = new CANSparkMax(Amp.RIGHT_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkPIDController rightPID = rightMotor.getPIDController();

  private final boolean manualTuning = false;
  private AmpState desiredState = AmpState.HOME;

  @SuppressWarnings("resource")
  public AmpSubsystem() {
    final var tab = Shuffleboard.getTab("Amp");
    SparkUtils.configureRel(leftMotor, (spark, encoder, pid) -> {
      spark.setInverted(Amp.LEFT_INVERTED);
      Amp.REV_CONVERSION_FACTOR.apply(encoder);
      pid.setP(Amp.LEFT_GAINS.P());
      pid.setP(Amp.LEFT_GAINS.I());
      pid.setP(Amp.LEFT_GAINS.D());
      pid.setOutputRange(Amp.MIN_OUTPUT, Amp.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
      spark.setSmartCurrentLimit(NeoMotor.NEO_550_CURRENT_LIMIT);
      spark.setIdleMode(CANSparkBase.IdleMode.kBrake);
    });
    tab.addNumber("Left Pos", leftEncoder::getPosition);
    tab.add("Left PID", new TunableSparkPIDController(leftPID, () -> desiredState.leftPos(), (leftPos) -> {
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new AmpState("Manual", leftPos, desiredState.rightPos());
      leftPID.setReference(leftPos, CANSparkBase.ControlType.kPosition);
    }));

    SparkUtils.configureRel(rightMotor, (spark, encoder, pid) -> {
      spark.setInverted(Amp.RIGHT_INVERTED);
      Amp.REV_CONVERSION_FACTOR.apply(encoder);
      pid.setP(Amp.RIGHT_GAINS.P());
      pid.setP(Amp.RIGHT_GAINS.I());
      pid.setP(Amp.RIGHT_GAINS.D());
      pid.setOutputRange(Amp.MIN_OUTPUT, Amp.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
      spark.setSmartCurrentLimit(NeoMotor.NEO_550_CURRENT_LIMIT);
      spark.setIdleMode(CANSparkBase.IdleMode.kBrake);
    });
    tab.addNumber("Right Pos", rightEncoder::getPosition);
    tab.add("Right PID", new TunableSparkPIDController(rightPID, () -> desiredState.rightPos(), (rightPos) -> {
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new AmpState("Manual", desiredState.leftPos(), rightPos);
      rightPID.setReference(rightPos, CANSparkBase.ControlType.kPosition);
    }));

    resetPosition();

    tab.addString("State", () -> desiredState.name() + " (" + desiredState.leftPos() + ", " +  desiredState.rightPos() + ")");
    Tabs.MATCH.addBoolean("Amp At Desired State", this::atDesiredState);
    Tabs.MATCH.addString("Amp State", () -> desiredState.name() + " (" + desiredState.leftPos() + ", " +  desiredState.rightPos() + ")");
  }

  public void setDesiredState(AmpState desiredState) {
    this.desiredState = desiredState;
    leftPID.setReference(desiredState.leftPos(), CANSparkBase.ControlType.kPosition);
    rightPID.setReference(desiredState.rightPos(), CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(leftEncoder.getPosition(), desiredState.leftPos(), Amp.POSITION_TOLERANCE) &&
            MathUtil.isNear(rightEncoder.getPosition(), desiredState.rightPos(), Amp.POSITION_TOLERANCE);
  }

  public void resetPosition() {
    SparkUtils.runBlockingRel(leftMotor, (a, encoder, b) -> {
      encoder.setPosition(0.0);
    });

    SparkUtils.runBlockingRel(rightMotor, (a, encoder, b) -> {
      encoder.setPosition(0.0);
    });
  }
}