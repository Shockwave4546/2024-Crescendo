package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IdleIntakeCommand extends Command {
  private final IntakeSubsystem intake;
  
  public IdleIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override public void execute() {
    if (!intake.isRunIdle()) return;
    intake.runIdle();
  }
}