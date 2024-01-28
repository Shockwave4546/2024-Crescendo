package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

import java.time.Duration;
import java.time.Instant;
import java.util.function.BooleanSupplier;

public class IntakeNoteCommand extends Command {
  private static final Duration POST_INTAKE_TIME = Duration.ofSeconds(2);
  private final BooleanSupplier cancelCondition;
  private final IntakeSubsystem intake;
  private Instant complete;

  public IntakeNoteCommand(BooleanSupplier cancelCondition, IntakeSubsystem intake) {
    this.cancelCondition = cancelCondition;
    this.intake = intake;
    addRequirements(intake);
  }

  @Override public void execute() {
    intake.runIntake(false);
  }

  @Override public void end(boolean interrupted) {
    if (complete == null) {
       complete = Instant.now().plus(POST_INTAKE_TIME);
    }

    while (Instant.now().isBefore(complete)) {
      if (cancelCondition.getAsBoolean()) {
        intake.stopIntake();
        break;
      }

      intake.runIntake(false);
    }

    intake.stopIntake();
    complete = null;
  }

  @Override public boolean isFinished() {
    return cancelCondition.getAsBoolean() || intake.hasNote();
  }
}