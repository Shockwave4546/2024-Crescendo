package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pose.VisionSubsystem;

public class ShootInterpolatedCommand extends Command {
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;

  public ShootInterpolatedCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
    this.shooter = shooter;
    this.vision = vision;
    addRequirements(shooter, vision);
  }

  @Override public void execute() {
    if (!vision.hasViableTarget()) {
      shooter.setFlapState(ShooterSubsystem.FlapState.SUBWOOFER);
      shooter.rampUp(ShooterSubsystem.ShotType.SUBWOOFER);
      return;
    }

    shooter.rampUp(ShooterSubsystem.ShotType.INTERPOLATED);
  }

  @Override public void end(boolean interrupted) {
    shooter.setFlapState(ShooterSubsystem.FlapState.HOME);
    shooter.stopMotors();
  }
}