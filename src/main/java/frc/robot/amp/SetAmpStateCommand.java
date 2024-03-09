package frc.robot.amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetAmpStateCommand extends InstantCommand {
  public SetAmpStateCommand(AmpSubsystem.State state, AmpSubsystem amp) {
    super(() -> amp.setDesiredState(state), amp);
  }
}