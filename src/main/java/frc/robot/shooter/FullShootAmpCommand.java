package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.utils.EndActionSequentialCommandGroup;

public class FullShootAmpCommand extends EndActionSequentialCommandGroup {
  public FullShootAmpCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm) {
    super(new ResetRobotStateCommand(shooter, intake, arm));
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(IntakeArmSubsystem.State.HOME), arm),
            new InstantCommand(() -> shooter.rampUp(ShooterSubsystem.ShotType.AMP), shooter),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(0.5),
            new FeedShooterCommand(intake).withTimeout(0.25),
            new InstantCommand(shooter::stopMotors, shooter)
    );

    addRequirements(shooter, intake, arm);
  }}