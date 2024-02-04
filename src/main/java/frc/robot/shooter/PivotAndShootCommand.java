package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;
import frc.robot.pose.VisionSubsystem;

public class PivotAndShootCommand extends SequentialCommandGroup {
  public PivotAndShootCommand(ShooterSubsystem shooter, VisionSubsystem vision, IntakeSubsystem intake, IntakeArmSubsystem arm) {
    addCommands(
            new ParallelCommandGroup(
                    new PivotIntakeCommand(IntakeArmSubsystem.State.IN, arm).until(arm::atDesiredState),
                    new ShootInterpolatedCommand(shooter, vision).until(shooter::atDesiredRPM)
            ),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(shooter, vision, intake, arm);
  }
}
