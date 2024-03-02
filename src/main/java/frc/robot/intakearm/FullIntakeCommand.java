package frc.robot.intakearm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.IntakeNoteCommand;
import frc.robot.intake.IntakeSubsystem;

public class FullIntakeCommand extends SequentialCommandGroup {
  public FullIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new PivotIntakeCommand(IntakeArmSubsystem.State.FLOOR, arm),
            new IntakeNoteCommand(() -> false, intake).until(intake::hasNote),
            new PivotIntakeCommand(IntakeArmSubsystem.State.HOME, arm)
    );

    addRequirements(arm, intake);
  }
}
