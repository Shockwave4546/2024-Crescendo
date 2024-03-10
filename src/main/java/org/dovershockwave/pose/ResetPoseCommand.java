package org.dovershockwave.pose;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.dovershockwave.swerve.SwerveSubsystem;

public class ResetPoseCommand extends InstantCommand {
  public ResetPoseCommand(SwerveSubsystem swerve, PoseEstimatorSubsystem poseEstimator) {
    super(() -> { swerve.zeroGyro(); poseEstimator.resetPose(); }, poseEstimator);
  }
}