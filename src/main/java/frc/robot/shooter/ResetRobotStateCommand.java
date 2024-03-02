package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.IntakeArmSubsystem;

public class ResetRobotStateCommand extends SequentialCommandGroup {
  public ResetRobotStateCommand(ShooterSubsystem shooter, IntakeSubsystem intake, IntakeArmSubsystem arm) {
    addCommands(
            new InstantCommand(() -> arm.setDesiredState(IntakeArmSubsystem.State.HOME), arm),
            new InstantCommand(shooter::stopMotors, shooter),
            new InstantCommand(intake::stopIntake, intake)
    );

    addRequirements(shooter, intake, arm);
  }
}