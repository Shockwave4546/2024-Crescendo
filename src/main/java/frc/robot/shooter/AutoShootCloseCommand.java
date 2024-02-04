package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;

public class AutoShootCloseCommand extends SequentialCommandGroup {
  public AutoShootCloseCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    addCommands(
            new ShootCloseCommand(shooter).until(shooter::atDesiredRPM),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(shooter, intake);
  }
}