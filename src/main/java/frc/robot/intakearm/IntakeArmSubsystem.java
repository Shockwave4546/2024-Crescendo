package frc.robot.intakearm;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArm;
import frc.robot.Constants.Tabs;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.TunableSparkPIDController;
import frc.robot.utils.SparkUtils;

public class IntakeArmSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(IntakeArm.MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
  private final SparkPIDController pid = motor.getPIDController();
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private final boolean manualTuning = false;
  private ArmState desiredState = ArmState.HOME;

  @SuppressWarnings("resource")
  public IntakeArmSubsystem() {
    SparkUtils.configureAbs(motor, (motor, encoder, pid) -> {
      motor.setSmartCurrentLimit(Constants.NeoMotor.NEO_CURRENT_LIMIT);
      motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      IntakeArm.ANGLE_CONVERSION_FACTOR.apply(encoder);
      encoder.setInverted(false);
      encoder.setZeroOffset(IntakeArm.ANGLE_OFFSET);

      pid.setP(IntakeArm.GAINS.P());
      pid.setI(IntakeArm.GAINS.I());
      pid.setD(IntakeArm.GAINS.D());
      pid.setOutputRange(IntakeArm.MIN_OUTPUT, IntakeArm.MAX_OUTPUT);
      pid.setFeedbackDevice(encoder);
    });

    final var tab = Shuffleboard.getTab("Intake Arm");
    tab.addNumber("Duty Cycle", motor::getAppliedOutput);
    tab.addNumber("Current Angle", encoder::getPosition);
    tab.add("PID", new TunableSparkPIDController(pid, () -> desiredState.angle(), (angle) -> {
      if (!manualTuning || RobotContainer.isCompetition()) return;
      this.desiredState = new ArmState("Manual", angle);
      pid.setReference(angle, CANSparkMax.ControlType.kPosition);
    }));
    tab.addString("State", () -> desiredState.name() + " (" + desiredState.angle() + "°)");
    Tabs.MATCH.addBoolean("Arm At Desired State", this::atDesiredState);
    Tabs.MATCH.addString("Arm State", () -> desiredState.name() + " (" + desiredState.angle() + "°)");
    Tabs.MATCH.addBoolean("Arm Valid Encoder", () -> !shouldStopArm());
  }

  @Override public void periodic() {
     if (shouldStopArm()) {
       DriverStation.reportError("The encoder is reporting an angle that will break the arm: " + encoder.getPosition(), false);
     }
  }

  public void setDesiredState(ArmState desiredState) {
    if (shouldStopArm()) return;
    this.desiredState = desiredState;
    pid.setReference(desiredState.angle(), CANSparkBase.ControlType.kPosition);
  }

  public boolean atDesiredState() {
    return MathUtil.isNear(encoder.getPosition(), desiredState.angle(), IntakeArm.ANGLE_TOLERANCE);
  }

  /**
   * If the Encoder is reading an angle that causes the arm to go into the robot, it should stop.
   * These angles include [198, 360].
   *
   * @return whether the arm should stop operating as to not break it.
   */
  private boolean shouldStopArm() {
    return encoder.getPosition() > IntakeArm.MAX_ANGLE;
  }
}