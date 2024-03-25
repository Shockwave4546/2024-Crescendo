package org.dovershockwave.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IdleShooterCommand extends Command {
  private final ShooterSubsystem shooter;

  public IdleShooterCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override public void execute() {
    if (!shooter.isRunIdle()) {
      shooter.stopMotors();
      return;
    }

    shooter.setDesiredState(ShooterState.IDLE);
  }
}