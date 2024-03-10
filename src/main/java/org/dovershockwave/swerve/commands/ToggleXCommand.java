package org.dovershockwave.swerve.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.dovershockwave.swerve.SwerveSubsystem;

public class ToggleXCommand extends InstantCommand {
  public ToggleXCommand(SwerveSubsystem swerve) {
    super(swerve::toggleX, swerve);
  }
}