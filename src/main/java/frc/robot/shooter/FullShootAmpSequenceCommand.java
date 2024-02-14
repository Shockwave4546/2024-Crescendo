package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.utils.EndActionSequentialCommandGroup;

public class FullShootAmpSequenceCommand extends EndActionSequentialCommandGroup {
  public FullShootAmpSequenceCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm) {
    super(new ResetRobotStateSequenceCommand(shooter, intake, arm));
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(IntakeArmSubsystem.State.HOME), arm),
            new InstantCommand(() -> shooter.rampUp(ShooterSubsystem.ShotType.AMP), shooter),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(1.0),
            new FeedShooterCommand(intake).withTimeout(1.0),
            new WaitCommand(0.5),
            new InstantCommand(shooter::stopMotors, shooter)
    );

    addRequirements(shooter, intake, arm);
  }
}