package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootCloseCommand extends Command {
  private final ShooterSubsystem shooter;

  public ShootCloseCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override public void execute() {
    shooter.setFlapState(ShooterSubsystem.FlapState.SUBWOOFER);
    shooter.rampUp(ShooterSubsystem.ShotType.SUBWOOFER);
  }

  @Override public void end(boolean interrupted) {
    shooter.setFlapState(ShooterSubsystem.FlapState.HOME);
    shooter.stopMotors();
  }
}