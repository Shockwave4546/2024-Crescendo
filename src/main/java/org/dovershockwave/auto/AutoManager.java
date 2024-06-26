package org.dovershockwave.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.dovershockwave.Constants;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.led.LEDSubsystem;
import org.dovershockwave.pose.PoseEstimatorSubsystem;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.shooter.ShooterState;
import org.dovershockwave.shooter.ShooterSubsystem;
import org.dovershockwave.shooterwrist.ShooterWristSubsystem;
import org.dovershockwave.shooterwrist.WristState;
import org.dovershockwave.swerve.SwerveSubsystem;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser;

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager(SwerveSubsystem swerve, PoseEstimatorSubsystem poseEstimator, LEDSubsystem led, ShooterSubsystem shooter, VisionSubsystem vision, IntakeSubsystem intake, IntakeArmSubsystem arm, ShooterWristSubsystem wrist) {
    AutoBuilder.configureHolonomic(
            poseEstimator::getPose2d,
            poseEstimator::resetOdometry,
            swerve::getRelativeChassisSpeed,
            swerve::driveAutonomous,
            new HolonomicPathFollowerConfig(
                    new PIDConstants(Constants.Auto.DRIVING_GAINS.P(), Constants.Auto.DRIVING_GAINS.I(), Constants.Auto.DRIVING_GAINS.D()),
                    new PIDConstants(Constants.Auto.TURNING_GAINS.P(), Constants.Auto.TURNING_GAINS.I(), Constants.Auto.TURNING_GAINS.D()),
                    Constants.Swerve.MAX_SPEED_METERS_PER_SECOND,
                    Constants.Swerve.WHEEL_BASE / 2,
                    new ReplanningConfig()
            ),
            this::shouldFlipPath,
            swerve
    );

    // Note: Named commands must be registered before the creation of any PathPlanner Autos or Paths.
    NamedCommands.registerCommand("RampClose", new InstantCommand(() -> shooter.setDesiredState(ShooterState.SUBWOOFER), shooter));
    NamedCommands.registerCommand("IntakeNote", new AutoIntakeCommand(arm, intake));
    NamedCommands.registerCommand("ShootClose", new AutoShootCloseCommand(intake, shooter, arm, wrist));
    NamedCommands.registerCommand("StopShooter", new InstantCommand(shooter::stopMotors, shooter));
    NamedCommands.registerCommand("IntakeHome", new InstantCommand(() -> arm.setDesiredState(ArmState.HOME), arm));
    NamedCommands.registerCommand("ShootInterpolated", new InstantCommand(() -> shooter.setDesiredState(ShooterState.INTERPOLATED), shooter));
    NamedCommands.registerCommand("WristHome", new InstantCommand(() -> wrist.setDesiredState(WristState.HOME), wrist));

    this.chooser = AutoBuilder.buildAutoChooser("Do nothing.");
    Constants.Tabs.MATCH.add("Autonomous", chooser).withSize(3, 3).withPosition(12, 0);
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