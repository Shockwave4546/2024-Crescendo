package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.shooter.ShooterSubsystem;

public class AutoShootCloseCommand extends SequentialCommandGroup {
  public AutoShootCloseCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm) {
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(IntakeArmSubsystem.State.HOME), arm),
            new WaitUntilCommand(arm::atDesiredState),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(0.5),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(intake, arm);
  }
}