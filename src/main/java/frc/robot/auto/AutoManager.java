package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Tabs;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem.State;
import frc.robot.led.LEDSubsystem;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.pose.VisionSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser;

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager(SwerveSubsystem swerve, PoseEstimatorSubsystem poseEstimator, LEDSubsystem led, ShooterSubsystem shooter, VisionSubsystem vision, IntakeSubsystem intake, IntakeArmSubsystem arm) {
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

    // Note: Named commands must be registered before the creation of any PathPlanner Autos or Paths.
    NamedCommands.registerCommand("RampClose", new InstantCommand(() -> shooter.rampUp(ShooterSubsystem.ShotType.SUBWOOFER)));
    NamedCommands.registerCommand("IntakeNote", new AutoIntakeCommand(arm, intake));
    NamedCommands.registerCommand("ShootClose", new AutoShootCloseCommand(intake, shooter, arm));
    NamedCommands.registerCommand("StopShooter", new InstantCommand(shooter::stopMotors, shooter));
    NamedCommands.registerCommand("IntakeHome", new InstantCommand(() -> arm.setDesiredState(State.HOME), arm));

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