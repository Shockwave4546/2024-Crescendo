package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.ArmState;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;
import frc.robot.shooter.ShooterSubsystem;

public class AutoShootCloseCommand extends SequentialCommandGroup {
  public AutoShootCloseCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm) {
    addCommands(
            new PivotIntakeCommand(ArmState.HOME, arm),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(0.75),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(intake, arm);
  }
}