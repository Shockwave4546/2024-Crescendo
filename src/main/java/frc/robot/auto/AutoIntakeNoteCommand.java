package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.intake.IntakeNoteCommand;
import frc.robot.intake.IntakeSubsystem;

public class AutoIntakeNoteCommand extends SequentialCommandGroup {
  private static final double INTAKE_OFFSET_SECS = 0.05;

  public AutoIntakeNoteCommand(IntakeSubsystem intake) {
    addCommands(
            new IntakeNoteCommand(() -> false, intake),
            new WaitCommand(IntakeNoteCommand.POST_INTAKE_TIME.getSeconds() + INTAKE_OFFSET_SECS)
    );

    addRequirements(intake);
  }
}