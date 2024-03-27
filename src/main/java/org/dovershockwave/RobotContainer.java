package org.dovershockwave;

import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.amp.AmpSubsystem;
import org.dovershockwave.auto.AutoManager;
import org.dovershockwave.intake.FeedShooterCommand;
import org.dovershockwave.intake.IntakeSubsystem;
import org.dovershockwave.intakearm.ArmState;
import org.dovershockwave.intakearm.FullIntakeCommand;
import org.dovershockwave.intakearm.IntakeArmSubsystem;
import org.dovershockwave.led.LEDSubsystem;
import org.dovershockwave.pose.PoseEstimatorSubsystem;
import org.dovershockwave.pose.ResetPoseCommand;
import org.dovershockwave.pose.VisionSubsystem;
import org.dovershockwave.shooter.*;
import org.dovershockwave.shooterwrist.ShooterWristSubsystem;
import org.dovershockwave.swerve.SwerveSubsystem;
import org.dovershockwave.swerve.commands.SetSpeedMaxCommand;
import org.dovershockwave.swerve.commands.ToggleXCommand;

import java.util.Map;

public class RobotContainer {
  protected final CommandXboxController driverController = new CommandXboxController(Constants.IO.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.IO.OPERATOR_CONTROLLER_PORT);
  protected final VisionSubsystem vision = new VisionSubsystem();
  protected final SwerveSubsystem swerve = new SwerveSubsystem();
  protected final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerve, vision);
  protected final IntakeSubsystem intake = new IntakeSubsystem();
  protected final ShooterSubsystem shooter = new ShooterSubsystem(vision);
  protected final IntakeArmSubsystem arm = new IntakeArmSubsystem();
  protected final LEDSubsystem led = new LEDSubsystem();
  protected final ShooterWristSubsystem wrist = new ShooterWristSubsystem(vision);
  protected final AutoManager auto = new AutoManager(swerve, poseEstimator, led, shooter, vision, intake, arm, wrist);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    vision.setPoseEstimator(poseEstimator);
    final UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    camera.setFPS(30);
    Constants.Tabs.MATCH.add("Front", camera)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withSize(12, 10)
            .withPosition(0, 0)
            .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

    configureButtonBindings();

     if (isCompetition()) {
       PPLibTelemetry.enableCompetitionMode();
     }

    shooter.setDefaultCommand(new IdleShooterCommand(shooter, intake));
  }

  public static boolean isCompetition() {
    return DriverStation.getMatchType() != DriverStation.MatchType.None;
  }

  public static int getSubwooferTagID() {
    return (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? 7 : 4;
  }

  private void configureButtonBindings() {
    driverController.b().onTrue(new ResetPoseCommand(swerve, poseEstimator));
    driverController.x().onTrue(new ToggleXCommand(swerve));
    driverController.leftBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.2, 0.2));
    driverController.rightBumper().whileTrue(new SetSpeedMaxCommand(swerve, 0.4, 0.4));

    operatorController.povUp().onTrue(new InstantCommand(() -> arm.setDesiredState(ArmState.HOME), arm));
    operatorController.povDown().onTrue(new InstantCommand(() -> arm.setDesiredState(ArmState.FLOOR), arm));
    operatorController.povRight().onTrue(new FeedShooterCommand(intake).withTimeout(0.25));

    operatorController.leftBumper().onTrue(new ResetRobotStateCommand(shooter, intake, arm, wrist));

    operatorController.a().toggleOnTrue(new FullShootCloseCommand(intake, shooter, arm, wrist).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    operatorController.b().toggleOnFalse(new FullShootInterpolatedCommand(intake, shooter, arm, wrist).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    operatorController.x().toggleOnTrue(new FullShootAmpCommand(intake, shooter, arm, wrist).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    operatorController.y().toggleOnTrue(new FullIntakeCommand(arm, intake).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
  }
}