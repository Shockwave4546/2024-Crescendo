package org.dovershockwave.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.intake.FeedShooterCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.shooterwrist.ShooterWristSubsystem;
import org.dovershockwave.shooterwrist.WristState;
import org.dovershockwave.utils.EndActionSequentialCommandGroup;

public class FullSpitCommand extends EndActionSequentialCommandGroup {
  public FullSpitCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm, ShooterWristSubsystem wrist) {
    super(new ResetRobotStateCommand(shooter, intake, arm, wrist));
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(ArmState.HOME), arm),
            new InstantCommand(() -> shooter.setDesiredState(ShooterState.SPIT), shooter),
            new InstantCommand(() -> wrist.setDesiredState(WristState.SPIT), wrist),
            new WaitUntilCommand(shooter::atDesiredState),
            new WaitUntilCommand(wrist::atDesiredState),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(shooter, intake, arm, wrist);
  }
}