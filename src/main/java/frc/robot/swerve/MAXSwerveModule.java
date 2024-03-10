package frc.robot.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.Module;
import frc.robot.shuffleboard.TunableSparkPIDController;
import frc.robot.utils.SparkUtils;

import static com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.NeoMotor;

public class MAXSwerveModule {
  private static final ShuffleboardTab TAB = Shuffleboard.getTab("Swerve");
  private static int COUNT = 0;
  
  private final CANSparkMax drivingMotor;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPID;
  private final SparkPIDController turningPID;

  private final double chassisAngularOffset;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and SparkMaxPIDController. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a ThroughBore Encoder.
   */
  @SuppressWarnings("resource")
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean invertDrivingDirection, String prefix) {
    this.drivingMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    this.drivingEncoder = drivingMotor.getEncoder();
    this.drivingPID = drivingMotor.getPIDController();

    SparkUtils.configureRel(drivingMotor, (motor, encoder, pid) -> {
      motor.setInverted(invertDrivingDirection);
      pid.setFeedbackDevice(encoder);

      Module.DRIVING_ENCODER_POSITION_FACTOR.apply(encoder);
      encoder.setVelocityConversionFactor(Module.DRIVING_ENCODER_VELOCITY_FACTOR);

      pid.setP(Module.DRIVING_GAINS.P());
      pid.setI(Module.DRIVING_GAINS.I());
      pid.setD(Module.DRIVING_GAINS.D());
      pid.setFF(Module.DRIVING_GAINS.FF());
      pid.setOutputRange(Module.DRIVING_MIN_OUTPUT, Module.DRIVING_MAX_OUTPUT);

      motor.setIdleMode(Module.DRIVING_MOTOR_IDLE_MODE);
      motor.setSmartCurrentLimit(NeoMotor.NEO_CURRENT_LIMIT);
    });

    final var turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);
    this.turningEncoder = turningMotor.getAbsoluteEncoder();
    this.turningPID = turningMotor.getPIDController();
    SparkUtils.configureAbs(turningMotor, (motor, encoder, pid) -> {
      pid.setFeedbackDevice(turningEncoder);

      Module.TURNING_ENCODER_POSITION_FACTOR.apply(encoder);
      encoder.setVelocityConversionFactor(Module.TURNING_ENCODER_VELOCITY_FACTOR);
      encoder.setInverted(Module.TURNING_ENCODER_INVERTED);

      pid.setPositionPIDWrappingEnabled(true);
      pid.setPositionPIDWrappingMinInput(Module.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
      pid.setPositionPIDWrappingMaxInput(Module.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

      pid.setP(Module.TURNING_GAINS.P());
      pid.setI(Module.TURNING_GAINS.I());
      pid.setD(Module.TURNING_GAINS.D());
      pid.setFF(Module.TURNING_GAINS.FF());
      pid.setOutputRange(Module.TURNING_MIN_OUTPUT, Module.TURNING_MAX_OUTPUT);

      motor.setIdleMode(Module.TURNING_MOTOR_IDLE_MODE);
      motor.setSmartCurrentLimit(NeoMotor.NEO_550_CURRENT_LIMIT);
    });

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0.0);

    final var colIndex = COUNT * 7;
    TAB.addNumber(prefix + drivingCANId + " Duty Cycle", drivingMotor::getAppliedOutput).withSize(4, 1).withPosition(colIndex, 0);
    TAB.addNumber(prefix + drivingCANId + " Position", drivingEncoder::getPosition).withSize(4, 1).withPosition(colIndex, 1);
    TAB.addNumber(prefix + drivingCANId + " Velocity", drivingEncoder::getVelocity).withSize(4, 1).withPosition(colIndex, 2);
    TAB.add(prefix + drivingCANId + " PID", new TunableSparkPIDController(drivingPID, () -> desiredState.speedMetersPerSecond, (val) -> setDesiredState(new SwerveModuleState(val, new Rotation2d())))).withSize(3, 3).withPosition(colIndex + 4, 0);

    TAB.addNumber(prefix + turningCANId + " Duty Cycle", turningMotor::getAppliedOutput).withSize(4, 1).withPosition(colIndex, 4);
    TAB.addNumber(prefix + turningCANId + " Angle", turningEncoder::getPosition).withSize(4, 1).withPosition(colIndex, 5);
    TAB.addNumber(prefix + turningCANId + " Angular Velocity", turningEncoder::getVelocity).withSize(4, 1).withPosition(colIndex, 6);
    TAB.add(prefix + turningCANId + " PID", new TunableSparkPIDController(turningPID, turningEncoder::getVelocity, (val) -> turningPID.setReference(val, ControlType.kVelocity))).withSize(3, 3).withPosition(colIndex + 4, 4);
    COUNT++;
  }

  /**
   * Returns the current state of the module. A chassis angular offset is applied to the encoder position
   * to get the position relative to the chassis.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drivingEncoder.getVelocity(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module. A chassis angular offset is applied to the encoder position
   * to get the position relative to the chassis.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    final var correctedDesiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))
    );

    // Optimize the reference state to avoid spinning further than 90 degrees.
    final var optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /**
   * Zeroes the SwerveModule encoder.
   */
  public void resetEncoders() {
    SparkUtils.runBlockingRel(drivingMotor, (a, encoder, b) -> encoder.setPosition(0.0));
  }
}
