package org.dovershockwave.intakearm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.intake.IntakeNoteCommand;
import org.dovershockwave.intake.IntakeSubsystem;

public class FullIntakeCommand extends SequentialCommandGroup {
  public FullIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(ArmState.FLOOR), arm),
            new IntakeNoteCommand(intake).until(intake::hasNote),
            new InstantCommand(() -> arm.setDesiredState(ArmState.HOME), arm)
    );

    addRequirements(intake, arm);
  }
}
