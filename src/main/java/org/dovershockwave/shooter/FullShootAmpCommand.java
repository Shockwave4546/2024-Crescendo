package org.dovershockwave.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.amp.AmpState;
import org.dovershockwave.amp.AmpSubsystem;
import org.dovershockwave.amp.SetAmpStateCommand;
import org.dovershockwave.intake.FeedShooterCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.intakearm.PivotIntakeCommand;
import org.dovershockwave.utils.EndActionSequentialCommandGroup;

public class FullShootAmpCommand extends EndActionSequentialCommandGroup {
  public FullShootAmpCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm, AmpSubsystem amp) {
    super(new ResetRobotStateCommand(shooter, intake, arm, amp));
    addCommands(
            new SetAmpStateCommand(AmpState.AMP, amp),
            new PivotIntakeCommand(ArmState.HOME, arm),
            new InstantCommand(() -> shooter.rampUp(ShooterState.AMP), shooter),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(0.75),
            new FeedShooterCommand(intake).withTimeout(0.25),
            new InstantCommand(shooter::stopMotors, shooter),
            new WaitCommand(0.50),
            new SetAmpStateCommand(AmpState.HOME, amp)
    );

    addRequirements(shooter, intake, arm);
  }
}