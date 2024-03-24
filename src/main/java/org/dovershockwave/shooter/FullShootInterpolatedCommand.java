package org.dovershockwave.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.dovershockwave.amp.AmpSubsystem;
import org.dovershockwave.intake.FeedShooterCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.intakearm.PivotIntakeCommand;
import org.dovershockwave.shooterwrist.ShooterWristSubsystem;
import org.dovershockwave.shooterwrist.WristState;
import org.dovershockwave.utils.EndActionSequentialCommandGroup;

public class FullShootInterpolatedCommand extends EndActionSequentialCommandGroup {
  public FullShootInterpolatedCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm, ShooterWristSubsystem wrist) {
    super(new ResetRobotStateCommand(shooter, intake, arm, wrist));
    addCommands(
            new PivotIntakeCommand(ArmState.HOME, arm),
            new InstantCommand(() -> shooter.rampUp(ShooterState.INTERPOLATED), shooter),
            new InstantCommand(() -> wrist.interpolate(), wrist),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(0.75),
            new FeedShooterCommand(intake).withTimeout(0.25),
            new InstantCommand(shooter::stopMotors, shooter),
            new InstantCommand(() -> wrist.setDesiredState(WristState.HOME), wrist)
    );

    addRequirements(shooter, intake, arm, wrist);
  }
}