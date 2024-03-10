package org.dovershockwave.amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetAmpStateCommand extends InstantCommand {
  public SetAmpStateCommand(AmpState state, AmpSubsystem amp) {
    super(() -> amp.setDesiredState(state), amp);
  }
}