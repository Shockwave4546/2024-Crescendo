package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.amp.AmpSubsystem;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.ArmState;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;
import frc.robot.utils.EndActionSequentialCommandGroup;

public class FullShootCloseCommand extends EndActionSequentialCommandGroup {
  public FullShootCloseCommand(IntakeSubsystem intake, ShooterSubsystem shooter, IntakeArmSubsystem arm, AmpSubsystem amp) {
    super(new ResetRobotStateCommand(shooter, intake, arm, amp));
    addCommands(
            new PivotIntakeCommand(ArmState.HOME, arm),
            new InstantCommand(() -> shooter.rampUp(ShooterState.SUBWOOFER), shooter),
            new WaitUntilCommand(shooter::atDesiredRPS),
            new WaitCommand(0.75),
            new FeedShooterCommand(intake).withTimeout(0.25),
            new InstantCommand(shooter::stopMotors, shooter)
    );

    addRequirements(shooter, intake, arm, amp);
  }
}