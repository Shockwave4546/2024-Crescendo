package org.dovershockwave.pose;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.Constants.Vision;
import org.dovershockwave.shuffleboard.ShuffleboardBoolean;
import org.dovershockwave.shuffleboard.ShuffleboardDouble;
import org.dovershockwave.swerve.SwerveSubsystem;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;

import java.io.UncheckedIOException;

import static org.dovershockwave.Constants.Swerve;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final ShuffleboardTab tab = Shuffleboard.getTab("Odometry");
    /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   * Source: <a href="https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java">...</a>
   */
  private static final Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   * Source: <a href="https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java">...</a>
   */
  private static final Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
          Swerve.DRIVE_KINEMATICS,
          new Rotation2d(),
          new SwerveModulePosition[] {
                  new SwerveModulePosition(0.0, new Rotation2d()),
                  new SwerveModulePosition(0.0, new Rotation2d()),
                  new SwerveModulePosition(0.0, new Rotation2d()),
                  new SwerveModulePosition(0.0, new Rotation2d())
          },
          new Pose2d(),
          STATE_STD_DEVS,
          VISION_MEASUREMENT_STD_DEVS
  );

  private final ShuffleboardBoolean useVisionMeasurement = new ShuffleboardBoolean(tab,"Use Vision Measurement", false).withSize(3, 2).withPosition(3, 0);
  private final PhotonPoseEstimator cameraPoseEstimator;
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;

  private final ShuffleboardDouble tagX = new ShuffleboardDouble(tab, "Tag X (m)", 0.0).withSize(3, 2).withPosition(0, 4);
  private final ShuffleboardDouble tagY = new ShuffleboardDouble(tab, "Tag Y (m)", 0.0).withSize(3, 2).withPosition(3, 4);
  private final ShuffleboardDouble tagDegrees = new ShuffleboardDouble(tab, "Tag Degrees", 0.0).withSize(3, 2).withPosition(6, 4);
  private final ShuffleboardDouble tagID = new ShuffleboardDouble(tab, "Tag ID", -1.0).withSize(3, 2).withPosition(9, 4);

  private double previousPipelineTimestamp = 0.0;

  @SuppressWarnings("resource")
  public PoseEstimatorSubsystem(SwerveSubsystem swerve, VisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;

    try {
      final var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      // Uses blue side as default in the event that the alliance color is null.
      final var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      layout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);

      this.cameraPoseEstimator = new PhotonPoseEstimator(
              layout,
              PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
              vision.getPhotonCamera(),
              Vision.ROBOT_TO_CAMERA
      );

      this.cameraPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    } catch (UncheckedIOException e) {
      throw new RuntimeException(e);
    }

    tab.addNumber("Pose X (m)", () -> getPose2d().getX()).withSize(3, 2).withPosition(0, 2);
    tab.addNumber("Pose Y (m)", () -> getPose2d().getY()).withSize(3, 2).withPosition(3, 2);
    tab.addNumber("Pose Degrees", () -> getPose2d().getRotation().getDegrees()).withSize(3, 2).withPosition(6, 2);
  }

  @Override public void periodic() {
    poseEstimator.update(swerve.getHeadingRotation2d(), swerve.getEstimatedPositions());

    final var currentTimestamp = vision.getLatestPipelineTimestamp();
    if (currentTimestamp == previousPipelineTimestamp) return;
    if (!vision.hasViableTarget()) {
      tagX.set(0.0);
      tagY.set(0.0);
      tagDegrees.set(0.0);
      tagID.set(-1.0);
      return;
    }

    previousPipelineTimestamp = currentTimestamp;

    final var tag = vision.getTag();
    if (tag == null) return;
    final var transform = tag.getBestCameraToTarget();
    if (transform == null) return;
    tagX.set(transform.getX());
    tagY.set(transform.getY());
    tagDegrees.set(transform.getRotation().toRotation2d().getDegrees());
    tagID.set(tag.getFiducialId());

    cameraPoseEstimator.update().ifPresent(estimatedPose -> {
      if (!useVisionMeasurement.get()) return;
      poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), currentTimestamp);
    });
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
            swerve.getHeadingRotation2d(),
            swerve.getEstimatedPositions(),
            pose);
  }

  /**
   * Zeros the heading. This sets the direction for field-centric driving.
   */
  public void resetPose() {
    resetOdometry(new Pose2d(new Translation2d(), new Rotation2d()));
  }

  /**
   * @return the Pose2d of the robot.
   */
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * @return the Pose3d of the robot, with the z-axis being the heading of the robot.
   */
  public Pose3d getPose3d() {
    return new Pose3d(
            getPose2d().getX(),
            getPose2d().getY(),
            0.0,
            new Rotation3d(0.0, 0.0, getPose2d().getRotation().getRadians())
    );
  }
}