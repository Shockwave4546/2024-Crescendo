package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ReverseIntakeCommand extends Command {
  private final IntakeSubsystem intake;

  public ReverseIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override public void execute() {
    intake.runIntake(true);
  }

  @Override public void end(boolean interrupted) {
    intake.stopIntake();
  }
}