package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.pose.PoseEstimatorSubsystem;

public class ResetPoseCommand extends InstantCommand {
  public ResetPoseCommand(PoseEstimatorSubsystem poseEstimator) {
    super(poseEstimator::resetPose, poseEstimator);
  }
}