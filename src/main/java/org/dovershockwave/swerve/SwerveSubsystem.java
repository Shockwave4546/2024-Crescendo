package org.dovershockwave.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.shuffleboard.ShuffleboardBoolean;
import org.dovershockwave.shuffleboard.ShuffleboardSpeed;

import static org.dovershockwave.Constants.Swerve;
import static org.dovershockwave.Constants.Tabs;

public class SwerveSubsystem extends SubsystemBase {
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
          Swerve.FRONT_LEFT_DRIVING_CAN_ID,
          Swerve.FRONT_LEFT_TURNING_CAN_ID,
          Swerve.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
          true,
          "FL"
  );

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
          Swerve.FRONT_RIGHT_DRIVING_CAN_ID,
          Swerve.FRONT_RIGHT_TURNING_CAN_ID,
          Swerve.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
          true,
          "FR"
  );

  private final MAXSwerveModule backLeft = new MAXSwerveModule(
          Swerve.BACK_LEFT_DRIVING_CAN_ID,
          Swerve.BACK_LEFT_TURNING_CAN_ID,
          Swerve.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
          true,
          "RL"
  );

  private final MAXSwerveModule backRight = new MAXSwerveModule(
          Swerve.BACK_RIGHT_DRIVING_CAN_ID,
          Swerve.BACK_RIGHT_TURNING_CAN_ID,
          Swerve.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
          true,
          "RR"
  );

  private final AHRS gyro = new AHRS();
  private final ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
  private final ShuffleboardSpeed driveSpeedMultiplier = new ShuffleboardSpeed(tab, "Drive Speed Multiplier", Swerve.DEFAULT_DRIVE_SPEED_MULTIPLIER)
          .withSize(5, 2).withPosition(0, 8);
  private final ShuffleboardSpeed rotSpeedMultiplier = new ShuffleboardSpeed(tab, "Rot Speed Multiplier", Swerve.DEFAULT_ROT_SPEED_MULTIPLIER)
          .withSize(5, 2).withPosition(5, 8);
  private final ShuffleboardBoolean isFieldRelative = new ShuffleboardBoolean(Tabs.MATCH, "Is Field Relative?", true)
          .withSize(3, 3).withPosition(18, 0);
  private final ShuffleboardBoolean isX = new ShuffleboardBoolean(Tabs.MATCH, "Is X?", false)
          .withSize(3, 3).withPosition(15, 0);

  public SwerveSubsystem() {
    Shuffleboard.getTab("Odometry").add("Gyro", gyro).withSize(3, 3).withPosition(9, 0);
    resetEncoders();
  }

  /**
   * It's important the SwerveModules are passed in with respect to the Kinematics construction.
   *
   * @return chassis speed relative to the robot.
   */
  public ChassisSpeeds getRelativeChassisSpeed() {
    return Swerve.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
    );
  }

  private void setX() {
    setModuleStates(
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    );
  }

  public void toggleX() {
    isX.set(!isX.get());
  }

  public boolean isFieldRelative() {
    return isFieldRelative.get();
  }

  public void setMaxSpeed(double drive, double rot) {
    driveSpeedMultiplier.set(MathUtil.clamp(drive, 0.0, 1.0));
    rotSpeedMultiplier.set(MathUtil.clamp(rot, 0.0, 1.0));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean useDefaultSpeeds) {
    if (isX.get()) {
      setX();
      return;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    final double xSpeedDelivered = xSpeed * Swerve.MAX_SPEED_METERS_PER_SECOND * (useDefaultSpeeds ? Swerve.DEFAULT_DRIVE_SPEED_MULTIPLIER : driveSpeedMultiplier.get());
    final double ySpeedDelivered = ySpeed * Swerve.MAX_SPEED_METERS_PER_SECOND * (useDefaultSpeeds ? Swerve.DEFAULT_DRIVE_SPEED_MULTIPLIER : driveSpeedMultiplier.get());
    final double rotDelivered = rotSpeed * Swerve.MAX_ANGULAR_SPEED * (useDefaultSpeeds ? Swerve.DEFAULT_ROT_SPEED_MULTIPLIER : rotSpeedMultiplier.get());

    final var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeadingRotation2d())
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Stops the robot.
   */
  public void stop() {
    drive(0.0, 0.0, 0.0, false, false);
  }

  /**
   * Overridden drive function for PathPlanner autonomous. It's also important to note that autonomous drives
   * given robot relative ChassisSpeeds (not field relative).
   *
   * @param speeds Speed to drive.
   */
  public void driveAutonomous(ChassisSpeeds speeds) {
    final var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * @return the SwerveModulePositions of the SwerveModules.
   */
  public SwerveModulePosition[] getEstimatedPositions() {
    return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
    };
  }

  /**
   * Sets the swerve ModuleStates. (FL, FR, BL, BR)
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState... desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /**
   *  Zeroes the gyro of the robot.
   */
  public void zeroGyro() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading as a Rotation2d.
   */
  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getRawAngleDegrees());
  }

  /**
   * The default SwerveMax template has an issue with inverting the Gyro, so the workaround is
   * manually negating the AHRS#getAngle. This function shouldn't get called by the user.
   *
   * @return The properly negated angle in degrees.
   */
  private double getRawAngleDegrees() {
    return (Swerve.GYRO_REVERSED ? -1.0 : 1.0) * gyro.getAngle();
  }
}
