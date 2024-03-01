package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IO;
import frc.robot.Constants.Tabs;
import frc.robot.auto.AutoManager;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.FullIntakeSequenceCommand;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;
import frc.robot.led.LEDSubsystem;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.pose.VisionSubsystem;
import frc.robot.shooter.*;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.commands.ResetPoseCommand;
import frc.robot.swerve.commands.SetSpeedMaxCommand;
import frc.robot.swerve.commands.ToggleXCommand;

public class RobotContainer {
  protected final CommandXboxController driverController = new CommandXboxController(IO.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(IO.OPERATOR_CONTROLLER_PORT);
  protected final VisionSubsystem vision = new VisionSubsystem();
  protected final SwerveSubsystem swerve = new SwerveSubsystem();
  protected final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerve, vision);
  protected final IntakeSubsystem intake = new IntakeSubsystem();
  protected final ShooterSubsystem shooter = new ShooterSubsystem(vision);
  protected final IntakeArmSubsystem arm = new IntakeArmSubsystem();
  protected final LEDSubsystem led = new LEDSubsystem();
  protected final AutoManager auto = new AutoManager(swerve, poseEstimator, led, shooter, vision, intake, arm);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    Tabs.MATCH.add("PDP", new PowerDistribution());
    vision.setPoseEstimator(poseEstimator);

    configureButtonBindings();

     if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
       PPLibTelemetry.enableCompetitionMode();
     }
  }

  private void configureButtonBindings() {
    driverController.b().onTrue(new ResetPoseCommand(swerve, poseEstimator));
    driverController.x().onTrue(new ToggleXCommand(swerve));
    driverController.leftBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.4, 0.6));
    driverController.rightBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.6, 0.8));

    operatorController.povUp().onTrue(new PivotIntakeCommand(IntakeArmSubsystem.State.HOME, arm));
    operatorController.povDown().onTrue(new PivotIntakeCommand(IntakeArmSubsystem.State.FLOOR, arm));

    operatorController.leftBumper().onTrue(new ResetRobotStateSequenceCommand(shooter, intake, arm));

    operatorController.a().toggleOnTrue(new FullShootCloseSequenceCommand(intake, shooter, arm));
    operatorController.b().toggleOnTrue(new FullShootInterpolatedSequenceCommand(intake, shooter, arm));
    operatorController.x().toggleOnTrue(new FullShootAmpSequenceCommand(intake, shooter, arm));
    operatorController.y().toggleOnTrue(new FullIntakeSequenceCommand(arm, intake));
  }
}