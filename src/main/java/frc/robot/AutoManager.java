package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Tabs;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser;

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager(SwerveSubsystem swerve, PoseEstimatorSubsystem poseEstimator) {
    AutoBuilder.configureHolonomic(
            poseEstimator::getPose2d,
            poseEstimator::resetOdometry,
            swerve::getRelativeChassisSpeed,
            swerve::driveAutonomous,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(Auto.DRIVING_GAINS.P, Auto.DRIVING_GAINS.I, Auto.DRIVING_GAINS.D),
                    new PIDConstants(Auto.TURNING_GAINS.P, Auto.TURNING_GAINS.I, Auto.TURNING_GAINS.D),
                    Swerve.MAX_SPEED_METERS_PER_SECOND,
                    Swerve.WHEEL_BASE / 2,
                    new ReplanningConfig()
            ),
            this::shouldFlipPath,
            swerve
    );

    this.chooser = AutoBuilder.buildAutoChooser("Do nothing.");
    Tabs.MATCH.add("Autonomous", chooser).withSize(3, 2).withPosition(3, 0);
  }

  /**
   * Red = flip, since PathPlanner uses blue as the default wall.
   *
   * @return whether the autonomous path should be flipped dependent on the alliance color.
   */
  private boolean shouldFlipPath() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  /**
   * Schedules the selected autonomous mode.
   */
  public void executeRoutine() {
    chooser.getSelected().schedule();
  }
}