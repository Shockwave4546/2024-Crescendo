package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.IntakeNoteCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem.State;

public class AutoIntakeCommand extends SequentialCommandGroup {
  public AutoIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(State.FLOOR)),
            new IntakeNoteCommand(() -> false, intake).until(intake::hasNote),
            new InstantCommand(() -> arm.setDesiredState(State.HOME))
    );

    addRequirements(intake);
  }
}
