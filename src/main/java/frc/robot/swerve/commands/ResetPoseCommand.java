package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class ResetPoseCommand extends InstantCommand {
  public ResetPoseCommand(SwerveSubsystem swerve, PoseEstimatorSubsystem poseEstimator) {
    super(() -> { swerve.zeroGyro(); poseEstimator.resetPose(); }, poseEstimator);
  }
}