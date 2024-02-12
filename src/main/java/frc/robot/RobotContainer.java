package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IO;
import frc.robot.Constants.Tabs;
import frc.robot.intake.FeedShooterCommand;
import frc.robot.intake.IntakeNoteCommand;
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
import frc.robot.swerve.commands.ToggleXCommand;

public class RobotContainer {
  // protected final VisionSubsystem vision = new VisionSubsystem();
  // protected final SwerveSubsystem swerve = new SwerveSubsystem();
  // protected final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerve, vision);
  protected final IntakeSubsystem intake = new IntakeSubsystem();
  protected final ShooterSubsystem shooter = new ShooterSubsystem();
  protected final IntakeArmSubsystem arm = new IntakeArmSubsystem();
  protected final LEDSubsystem led = new LEDSubsystem();
  protected final CommandXboxController driverController = new CommandXboxController(IO.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(IO.OPERATOR_CONTROLLER_PORT);
  // protected final AutoManager auto = new AutoManager(swerve, poseEstimator, led, shooter, vision, intake, arm);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    Tabs.MATCH.add("PDP", new PowerDistribution());
    CameraServer.startAutomaticCapture();
    // vision.setPoseEstimator(poseEstimator);

    configureButtonBindings();

     if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
       PPLibTelemetry.enableCompetitionMode();
     }
  }

  private void configureButtonBindings() {
    // driverController.b().onTrue(new ResetPoseCommand(poseEstimator));
    // driverController.x().onTrue(new ToggleXCommand(swerve));

    operatorController.povUp().onTrue(new PivotIntakeCommand(IntakeArmSubsystem.State.HOME, arm));
    operatorController.povDown().onTrue(new PivotIntakeCommand(IntakeArmSubsystem.State.FLOOR, arm));

    operatorController.a().toggleOnTrue(new FullShootCloseSequenceCommand(intake, shooter));
    operatorController.b().toggleOnTrue(new FullShootInterpolatedSequenceCommand(intake, shooter));
    operatorController.x().toggleOnTrue(new FullIntakeSequenceCommand(arm, intake));
  }
}