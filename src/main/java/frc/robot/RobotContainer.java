package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IO;
import frc.robot.Constants.Tabs;
import frc.robot.amp.AmpSubsystem;
import frc.robot.auto.AutoManager;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intakearm.ArmState;
import frc.robot.intakearm.FullIntakeCommand;
import frc.robot.intakearm.IntakeArmSubsystem;
import frc.robot.intakearm.PivotIntakeCommand;
import frc.robot.led.LEDSubsystem;
import frc.robot.pose.PoseEstimatorSubsystem;
import frc.robot.pose.ResetPoseCommand;
import frc.robot.pose.VisionSubsystem;
import frc.robot.shooter.FullShootAmpCommand;
import frc.robot.shooter.FullShootCloseCommand;
import frc.robot.shooter.ResetRobotStateCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.SwerveSubsystem;
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
  protected final AmpSubsystem amp = new AmpSubsystem();
  protected final LEDSubsystem led = new LEDSubsystem();
  protected final AutoManager auto = new AutoManager(swerve, poseEstimator, led, shooter, vision, intake, arm);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    Tabs.MATCH.add("PDP", new PowerDistribution());
    vision.setPoseEstimator(poseEstimator);
    CameraServer.startAutomaticCapture();

    configureButtonBindings();

     if (isCompetition()) {
       PPLibTelemetry.enableCompetitionMode();
     }
  }

  public static boolean isCompetition() {
    return DriverStation.getMatchType() != DriverStation.MatchType.None;
  }

  private void configureButtonBindings() {
    driverController.b().onTrue(new ResetPoseCommand(swerve, poseEstimator));
    driverController.x().onTrue(new ToggleXCommand(swerve));
    driverController.leftBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.2, 0.2));
    driverController.rightBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.4, 0.4));

    operatorController.povUp().onTrue(new PivotIntakeCommand(ArmState.HOME, arm));
    operatorController.povDown().onTrue(new PivotIntakeCommand(ArmState.FLOOR, arm));

    operatorController.leftBumper().onTrue(new ResetRobotStateCommand(shooter, intake, arm, amp));

    operatorController.a().toggleOnTrue(new FullShootCloseCommand(intake, shooter, arm, amp));
    operatorController.x().toggleOnTrue(new FullShootAmpCommand(intake, shooter, arm, amp));
    operatorController.y().toggleOnTrue(new FullIntakeCommand(arm, intake));
  }
}