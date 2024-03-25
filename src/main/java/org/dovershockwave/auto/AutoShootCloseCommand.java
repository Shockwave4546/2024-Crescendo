package org.dovershockwave.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.intake.FeedShooterCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.shooter.ShooterSubsystem;
import org.dovershockwave.shooterwrist.ShooterWristSubsystem;
import org.dovershockwave.shooterwrist.WristState;

public class AutoShootCloseCommand extends SequentialCommandGroup {
  public AutoShootCloseCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm, ShooterWristSubsystem wrist) {
    addCommands(
            new InstantCommand(() -> wrist.setDesiredState(WristState.SUBWOOFER), wrist),
            new InstantCommand(() -> arm.setDesiredState(ArmState.HOME), arm),
            new WaitUntilCommand(shooter::atDesiredState),
            new WaitCommand(0.75),
            new FeedShooterCommand(intake).withTimeout(0.25)
    );

    addRequirements(intake, arm, wrist);
  }
}