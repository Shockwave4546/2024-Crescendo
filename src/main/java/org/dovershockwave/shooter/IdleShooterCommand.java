package org.dovershockwave.shooter;

import org.dovershockwave.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IdleShooterCommand extends Command {
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;

  public IdleShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter);
  }

  @Override public void execute() {
    if (!shooter.isRunIdle() || !intake.hasNote()) {
      shooter.stopMotors();
      return;
    }

    shooter.setDesiredState(ShooterState.IDLE);
  }
}