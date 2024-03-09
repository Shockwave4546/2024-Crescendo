package frc.robot.intakearm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.IntakeNoteCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem.State;

public class FullIntakeCommand extends SequentialCommandGroup {
  public FullIntakeCommand(IntakeArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(State.FLOOR), arm),
            new IntakeNoteCommand(() -> false, intake).until(intake::hasNote),
            new InstantCommand(() -> arm.setDesiredState(State.HOME), arm)
    );

    addRequirements(intake, arm);
  }
}
