package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.amp.AmpState;
import frc.robot.amp.AmpSubsystem;
import frc.robot.amp.SetAmpStateCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.ArmState;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;

public class ResetRobotStateCommand extends SequentialCommandGroup {
  public ResetRobotStateCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IntakeArmSubsystem arm, AmpSubsystem amp) {
    addCommands(
            new PivotIntakeCommand(ArmState.HOME, arm),
            new InstantCommand(shooter::stopMotors, shooter),
            new InstantCommand(intake::stopIntake, intake),
            new SetAmpStateCommand(AmpState.HOME, amp)
    );

    addRequirements(shooter, intake, arm);
  }
}