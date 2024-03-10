package org.dovershockwave.amp;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.shuffleboard.TunableSparkPIDController;
import org.dovershockwave.utils.SparkUtils;
import org.dovershockwave.Constants;

public class AmpSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.Amp.LEFT_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final SparkPIDController leftPID = leftMotor.getPIDController();

  private final CANSparkMax rightMotor = new CANSparkMax(Constants.Amp.RIGHT_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkPIDController rightPID = rightMotor.getPIDController();

  private final boolean manualTuning = false;
  private AmpState desiredState = AmpState.HOME;

  @SuppressWarnings("resource")
  public AmpSubsystem() {
    final var tab = Shuffleboard.getTab("Amp");
    SparkUtils.configureRel(leftMotor, (spark, encoder, pid) -> {
      spark.setInverted(Constants.Amp.LEFT_INVERTED);
      Constants.Amp.REV_CONVERSION_FACTOR.apply(encoder);
      pid.setP(Constants.Amp.LEFT_GAINS.P());
      pid.setP(Constants.Amp.LEFT_GAINS.I());
      pid.setP(Constants.Amp.LEFT_GAINS.D());
      pid.setOutputRange(Constants.Amp.MIN_OUTPUT, Constants.Amp.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
      spark.setSmartCurrentLimit(Constants.NeoMotor.NEO_550_CURRENT_LIMIT);
      spark.setIdleMode(CANSparkBase.IdleMode.kBrake);
    });
    tab.addNumber("Left Pos", leftEncoder::getPosition);
    tab.add("Left PID", new TunableSparkPIDController(leftPID, () -> desiredState.leftPos(), (leftPos) -> {
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new AmpState("Manual", leftPos, desiredState.rightPos());
      leftPID.setReference(leftPos, CANSparkBase.ControlType.kPosition);
    }));

    SparkUtils.configureRel(rightMotor, (spark, encoder, pid) -> {
      spark.setInverted(Constants.Amp.RIGHT_INVERTED);
      Constants.Amp.REV_CONVERSION_FACTOR.apply(encoder);
      pid.setP(Constants.Amp.RIGHT_GAINS.P());
      pid.setP(Constants.Amp.RIGHT_GAINS.I());
      pid.setP(Constants.Amp.RIGHT_GAINS.D());
      pid.setOutputRange(Constants.Amp.MIN_OUTPUT, Constants.Amp.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
      spark.setSmartCurrentLimit(Constants.NeoMotor.NEO_550_CURRENT_LIMIT);
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
    tab.add("Reset Pos", new InstantCommand(this::resetPosition, this));
    Constants.Tabs.MATCH.addBoolean("Amp At Desired State", this::atDesiredState).withSize(3, 3).withPosition(18, 3);
    Constants.Tabs.MATCH.addString("Amp State", () -> desiredState.name() + " (" + desiredState.leftPos() + ", " +  desiredState.rightPos() + ")").withSize(3, 3).withPosition(18, 6);
  }

  public void setDesiredState(AmpState desiredState) {
    this.desiredState = desiredState;
    leftPID.setReference(desiredState.leftPos(), CANSparkBase.ControlType.kPosition);
    rightPID.setReference(desiredState.rightPos(), CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(leftEncoder.getPosition(), desiredState.leftPos(), Constants.Amp.POSITION_TOLERANCE) &&
            MathUtil.isNear(rightEncoder.getPosition(), desiredState.rightPos(), Constants.Amp.POSITION_TOLERANCE);
  }

  public void resetPosition() {
    SparkUtils.runBlockingRel(leftMotor, (a, encoder, b) -> encoder.setPosition(0.0));
    SparkUtils.runBlockingRel(rightMotor, (a, encoder, b) -> encoder.setPosition(0.0));
  }
}