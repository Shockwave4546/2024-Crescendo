package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootFarCommand extends Command {
  private final ShooterSubsystem shooter;

  public ShootFarCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override public void execute() {
    shooter.disengageServo();
    shooter.shootFar();
  }

  @Override public void end(boolean interrupted) {
    shooter.stopMotors();
  }
}