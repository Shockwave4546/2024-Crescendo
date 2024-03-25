package org.dovershockwave.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.dovershockwave.intake.IntakeNoteCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;

public class AutoIntakeCommand extends SequentialCommandGroup {
  public AutoIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(ArmState.FLOOR)),
            new IntakeNoteCommand(intake).until(intake::hasNote),
            new InstantCommand(() -> arm.setDesiredState(ArmState.HOME))
    );

    addRequirements(intake);
  }
}
