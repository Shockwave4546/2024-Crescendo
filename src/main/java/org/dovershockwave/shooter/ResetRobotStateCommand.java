package org.dovershockwave.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.amp.AmpState;
import org.dovershockwave.amp.AmpSubsystem;
import org.dovershockwave.amp.SetAmpStateCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.intakearm.PivotIntakeCommand;
import org.dovershockwave.shooterwrist.ShooterWristSubsystem;
import org.dovershockwave.shooterwrist.WristState;

public class ResetRobotStateCommand extends SequentialCommandGroup {
  public ResetRobotStateCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IntakeArmSubsystem arm, ShooterWristSubsystem wrist) {
    addCommands(
            new PivotIntakeCommand(ArmState.HOME, arm),
            new InstantCommand(shooter::stopMotors, shooter),
            new InstantCommand(intake::stopIntake, intake),
            new InstantCommand(() -> wrist.setDesiredState(WristState.HOME), wrist)
    );

    addRequirements(shooter, intake, arm, wrist);
  }
}