package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;

public class FullShootCloseSequenceCommand extends SequentialCommandGroup {
  public FullShootCloseSequenceCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
        new InstantCommand(shooter::shootClose, shooter),
        // new InstantCommand(() -> shooter.setFlapState(ShooterSubsystem.FlapState.SUBWOOFER), shooter),
        new WaitUntilCommand(shooter::atDesiredRPS),
        new WaitCommand(1.0),
        new FeedShooterCommand(intake).withTimeout(1.0),
        new WaitCommand(0.5),
        new InstantCommand(shooter::stopMotors, shooter)
        // new InstantCommand(() -> shooter.setFlapState(ShooterSubsystem.FlapState.HOME), shooter)
    );

    addRequirements(shooter, intake);
  }
}