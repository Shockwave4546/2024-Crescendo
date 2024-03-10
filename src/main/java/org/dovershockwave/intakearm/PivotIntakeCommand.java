package org.dovershockwave.intakearm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PivotIntakeCommand extends InstantCommand {
  public PivotIntakeCommand(ArmState state, IntakeArmSubsystem arm) {
    super(() -> arm.setDesiredState(state), arm);
  }
}