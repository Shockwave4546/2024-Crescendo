package org.dovershockwave.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.Constants;
import org.dovershockwave.RobotContainer;
import org.dovershockwave.Constants.Swerve;
import org.dovershockwave.pose.PoseEstimatorSubsystem;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.swerve.SwerveSubsystem;

public class ChaseTagCommand extends Command {
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0.0, 0.0, new TrapezoidProfile.Constraints(Swerve.MAX_ANGULAR_SPEED, 4));
  private final VisionSubsystem vision;
  private final PoseEstimatorSubsystem poseEstimator;
  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;

  public ChaseTagCommand(VisionSubsystem vision, PoseEstimatorSubsystem poseEstimator, SwerveSubsystem swerve, CommandXboxController controller) {
    this.vision = vision;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.controller = controller;

    omegaController.setTolerance(5);
    omegaController.enableContinuousInput(-180, 180);

    addRequirements(vision, poseEstimator, swerve);
  }

  @Override public void initialize() {
    omegaController.reset(poseEstimator.getPose2d().getRotation().getRadians());
  }

  @Override public void execute() {
    System.out.println(omegaController.atGoal());
    final var tag = vision.getTag(RobotContainer.getSubwooferTagID());
    if (tag == null) {
      System.out.println("Debug 2");
      swerve.drive(
              -MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND),
              -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND),
              -MathUtil.applyDeadband(controller.getRightX(), Constants.IO.DRIVE_DEADBAND),
              swerve.isFieldRelative(),
              false
      );
      return;
    }

    final var tagAngle = 10 - -(180 - Math.abs(tag.getBestCameraToTarget().getRotation().toRotation2d().getDegrees()));
    final var robotAngle = swerve.getHeadingRotation2d().getDegrees();

    System.out.println("Tag Degree: " + tagAngle);
    System.out.println("Robot Degree: " + robotAngle);
    omegaController.setGoal(tagAngle);

     var rotSpeed = omegaController.atGoal() ? 0 : Math.toRadians(-omegaController.calculate(robotAngle));

    // swerve.drive(
    //         -MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND),
    //         -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND),
    //         -MathUtil.applyDeadband(controller.getRightX(), Constants.IO.DRIVE_DEADBAND),
    //         swerve.isFieldRelative(),
    //         false
    // );

    if (omegaController.atGoal() || tag == null) {
      rotSpeed = 0.0;
      return;
    }

    System.out.println("Rot Speed: " + rotSpeed);
   swerve.drive(
           -MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND),
           -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND),
           -rotSpeed,
           swerve.isFieldRelative(),
           false
   );
  }

  public double normalize(double degrees) {
    return (degrees + 450.0) % 360.0;
  }
}
