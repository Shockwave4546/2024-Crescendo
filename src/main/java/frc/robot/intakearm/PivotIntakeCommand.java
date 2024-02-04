package frc.robot.intakearm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PivotIntakeCommand extends InstantCommand {
  public PivotIntakeCommand(IntakeArmSubsystem.State state, IntakeArmSubsystem arm) {
    super(() -> arm.setDesiredState(state), arm);
  }
}
