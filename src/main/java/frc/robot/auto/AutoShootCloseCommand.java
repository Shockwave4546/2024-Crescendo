package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.ShootCloseCommand;
import frc.robot.shooter.ShooterSubsystem;

public class AutoShootCloseCommand extends SequentialCommandGroup {
  public AutoShootCloseCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    addCommands(
            new ShootCloseCommand(shooter).until(shooter::atDesiredRPS),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(shooter, intake);
  }
}