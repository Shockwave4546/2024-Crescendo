package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShootCloseCommand extends Command {
  private final ShooterSubsystem shooter;

  public ShootCloseCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override public void execute() {
    shooter.engageServo();
    shooter.setVelocity(Constants.Shooter.CLOSE_VELOCITY);
  }

  @Override public void end(boolean interrupted) {
    shooter.disengageServo();
    shooter.stopMotors();
  }
}