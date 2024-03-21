package org.dovershockwave.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.Constants;
import org.dovershockwave.swerve.SwerveSubsystem;

public class SwerveDriveCommand extends Command {
  private final CommandXboxController controller;
  private final SwerveSubsystem swerve;

  public SwerveDriveCommand(CommandXboxController controller, SwerveSubsystem drive) {
    this.controller = controller;
    this.swerve = drive;
    addRequirements(drive);
  }

  @Override public void execute() {
    swerve.drive(
            -MathUtil.applyDeadband(controller.getLeftY(), Constants.IO.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(controller.getLeftX(), Constants.IO.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(controller.getRightX(), Constants.IO.DRIVE_DEADBAND),
            swerve.isFieldRelative(),
            false
    );
  }
}
