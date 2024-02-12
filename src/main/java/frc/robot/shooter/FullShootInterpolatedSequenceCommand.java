package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;

public class FullShootInterpolatedSequenceCommand extends SequentialCommandGroup {
  public FullShootInterpolatedSequenceCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
            new ParallelCommandGroup(
                    new InstantCommand(shooter::shootInterpolated, shooter),
                    new InstantCommand(() -> shooter.setFlapState(ShooterSubsystem.FlapState.HOME), shooter),
                    new SequentialCommandGroup(
                            new WaitUntilCommand(shooter::atDesiredRPS),
                            new FeedShooterCommand(intake).withTimeout(0.25),
                            new InstantCommand(shooter::stopMotors, shooter)
                    )
            )
    );

    addRequirements(shooter, intake);
  }
}