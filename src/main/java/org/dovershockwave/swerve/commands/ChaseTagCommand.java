package org.dovershockwave.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.dovershockwave.pose.PoseEstimatorSubsystem;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.swerve.SwerveSubsystem;

import static org.dovershockwave.Constants.Swerve;

import org.dovershockwave.Constants;
import org.dovershockwave.Constants.Swerve;

public class ChaseTagCommand extends Command {
  private static final int TAG_TO_CHASE = 1;
  private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(2, 0.0, 0.0), new Rotation3d());
  private final ProfiledPIDController omegaController = new ProfiledPIDController(0.1, 0.0, 0.0,  new TrapezoidProfile.Constraints(Swerve.MAX_ANGULAR_SPEED, 4));
  private final VisionSubsystem vision;
  private final PoseEstimatorSubsystem poseEstimator;
  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;

  public ChaseTagCommand(VisionSubsystem vision, PoseEstimatorSubsystem poseEstimator, SwerveSubsystem swerve, CommandXboxController controller) {
    this.vision = vision;
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.controller = controller;

    omegaController.setTolerance(Units.degreesToRadians(10));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(vision, poseEstimator, swerve);
  }

  @Override public void initialize() {
    omegaController.reset(poseEstimator.getPose2d().getRotation().getRadians());
  }

  @Override public void execute() {
        System.out.println(omegaController.atGoal());
    if (!vision.hasViableTarget()) {
      swerve.drive(
        -MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(controller.getRightX(), Constants.IO.DRIVE_DEADBAND),
        swerve.isFieldRelative(),
        false
      );
      return;
    }


    if (vision.hasViableTarget() && vision.getTag().getFiducialId() != TAG_TO_CHASE) return;
    final var tagPose = vision.getTagRelativeToCenterPose();
    final var goalPose = tagPose.transformBy(TAG_TO_GOAL).toPose2d();
    // final var robotPose = poseEstimator.getPose2d();
    final var robotAngle = swerve.getHeadingRotation2d().getRadians();

    System.out.println("1:" + goalPose.getRotation().getRadians());
    System.out.println("2: " + robotAngle);
    omegaController.setGoal(goalPose.getRotation().getRadians());

    final var rotSpeed = omegaController.atGoal() ? 0 : -omegaController.calculate(robotAngle - goalPose.getRotation().getRadians());

          swerve.drive(
        -MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND),
        -MathUtil.applyDeadband(controller.getRightX(), Constants.IO.DRIVE_DEADBAND),
        swerve.isFieldRelative(),
        false
      );

    // swerve.drive(-MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND), 
    // -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND), 
    //   -rotSpeed, true, false);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }
}
