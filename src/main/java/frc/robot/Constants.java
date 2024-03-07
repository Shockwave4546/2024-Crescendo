package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.swerve.ModulePosition;
import frc.robot.utils.PIDGains;
import frc.robot.utils.PositionConversionFactor;

import java.util.Map;

public final class Constants {
  public static final class IntakeArm {
    public static final int MOTOR_CAN_ID = 33;
    public static final PositionConversionFactor ANGLE_CONVERSION_FACTOR = new PositionConversionFactor(PositionConversionFactor.ConversionType.DEGREES);
    public static final PIDGains GAINS = new PIDGains(0.01, 0.0, 0.005);
    public static final double MIN_OUTPUT = -1.0;
    public static final double MAX_OUTPUT = 1.0;
    public static final double ANGLE_TOLERANCE = 5.0; // degrees
    public static final double ANGLE_OFFSET = 192.0; // degrees
  }

  public static final class LED {
    public static final int PWM_ID = 0;
    public static final int BUFFER_LENGTH = 125;

    public static final Color RED = new Color(255, 0, 0);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color OFF = new Color(0, 0, 0);
    public static final Color WHITE = new Color(255, 255, 255);
  }

  public static final class Intake {
    public static final int LIMIT_SWITCH_DIO_PORT = 4;
    public static final int MOTOR_CAN_ID = 32;

    public static final double FORWARD_INTAKE_SPEED = -1.0;
    public static final double REVERSE_INTAKE_SPEED = 1.0;
  }

  public static final class Shooter {
    public static final int BOTTOM_CAN_ID = 31;
    public static final int TOP_CAN_ID = 30;

    public static final PositionConversionFactor REV_CONVERSION_FACTOR = new PositionConversionFactor(1);
    public static final float RPS_CONVERSION_FACTOR = 1 / 60F;

    public static final PIDGains GAINS = new PIDGains(0.03);
    public static final double MIN_OUTPUT = -1.0;
    public static final double MAX_OUTPUT = 1.0;
    public static final float FF = 0.011F;
    public static final double RPS_TOLERANCE = 7.5;
  }

  public static final class Swerve {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.65;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = 0.545; // m
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = 0.545; // m

    public static final Map<ModulePosition, Translation2d> MODULE_TRANSLATIONS = Map.of(
            ModulePosition.FRONT_LEFT, new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ModulePosition.FRONT_RIGHT, new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            ModulePosition.BACK_LEFT, new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            ModulePosition.BACK_RIGHT, new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            MODULE_TRANSLATIONS.get(ModulePosition.FRONT_LEFT),
            MODULE_TRANSLATIONS.get(ModulePosition.FRONT_RIGHT),
            MODULE_TRANSLATIONS.get(ModulePosition.BACK_LEFT),
            MODULE_TRANSLATIONS.get(ModulePosition.BACK_RIGHT)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = 0.0;

    // Driving Motor Prefix = 1x
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 10;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 11;
    public static final int BACK_LEFT_DRIVING_CAN_ID = 13;
    public static final int BACK_RIGHT_DRIVING_CAN_ID = 14;

    // Turning Motor Prefix = 2x
    public static final int FRONT_LEFT_TURNING_CAN_ID = 20;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 21;
    public static final int BACK_LEFT_TURNING_CAN_ID = 23;
    public static final int BACK_RIGHT_TURNING_CAN_ID = 24; 

    public static final boolean GYRO_REVERSED = true;

    public static final double DEFAULT_DRIVE_SPEED_MULTIPLIER = 0.85;
    public static final double DEFAULT_ROT_SPEED_MULTIPLIER = 0.85;
  }

  public static final class Vision {
    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

    public static final double MAXIMUM_AMBIGUITY = 0.2;
    public static final String FRONT_CAMERA_NAME = "OV9281";
  }

  public static final class Module {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotor.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.071;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
            / DRIVING_MOTOR_REDUCTION;

    public static final PositionConversionFactor DRIVING_ENCODER_POSITION_FACTOR =
            new PositionConversionFactor((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION); // Meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION) / 60.0; // Meters per second

    public static final PositionConversionFactor TURNING_ENCODER_POSITION_FACTOR =
            new PositionConversionFactor(PositionConversionFactor.ConversionType.RADIANS); // Radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Radians per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // Radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = 2 * Math.PI; // Radians

    public static final PIDGains DRIVING_GAINS = new PIDGains(0.15, 0.0, 0.02);
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT = -1;
    public static final double DRIVING_MAX_OUTPUT = 1;

    public static final PIDGains TURNING_GAINS = new PIDGains(0.20, 0.0, 0.00);
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT = -1;
    public static final double TURNING_MAX_OUTPUT = 1;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // Amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // Amps
  }

  public static final class Auto {
    public static final PIDGains DRIVING_GAINS = new PIDGains(1.5, 0.0, 0.08);

    public static final PIDGains TURNING_GAINS = new PIDGains(9.5, 0.0, 0.0);
  }

  public static final class IO {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.02;
  }

  public static final class NeoMotor {
    public static final double FREE_SPEED_RPM = 5676;
  }

  public static final class Tabs {
    public static final ShuffleboardTab MATCH = Shuffleboard.getTab("Match");
  }
}
