package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.IntakeNoteCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.ArmState;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;

public class AutoIntakeCommand extends SequentialCommandGroup {
  public AutoIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new PivotIntakeCommand(ArmState.FLOOR, arm),
            new IntakeNoteCommand(() -> false, intake).until(intake::hasNote),
            new PivotIntakeCommand(ArmState.HOME, arm)
    );

    addRequirements(intake);
  }
}
