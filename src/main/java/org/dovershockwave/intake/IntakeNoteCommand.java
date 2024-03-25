package org.dovershockwave.intake;

import edu.wpi.first.wpilibj2.command.Command;

import java.time.Duration;
import java.time.Instant;
import java.util.function.BooleanSupplier;

public class IntakeNoteCommand extends Command {
  private final IntakeSubsystem intake;

  public IntakeNoteCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override public void execute() {
    intake.runIntake(false);
  }

  @Override public void end(boolean interrupted) {
    intake.stopIntake();
  }

  @Override public boolean isFinished() {
    return intake.hasNote();
  }
}