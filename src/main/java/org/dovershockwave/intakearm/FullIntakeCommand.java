package org.dovershockwave.intakearm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.intake.IntakeNoteCommand;
import org.dovershockwave.intake.IntakeSubsystem;

public class FullIntakeCommand extends SequentialCommandGroup {
  public FullIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new PivotIntakeCommand(ArmState.FLOOR, arm),
            new IntakeNoteCommand(() -> false, intake).until(intake::hasNote),
            new PivotIntakeCommand(ArmState.HOME, arm)
    );

    addRequirements(intake, arm);
  }
}
