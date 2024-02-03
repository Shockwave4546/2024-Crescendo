package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swerve.commands.SwerveDriveCommand;

public class Robot extends TimedRobot {
  private RobotContainer container;

  @Override public void robotInit() {
    container = new RobotContainer();
  }

  @Override public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override public void autonomousInit() {
    container.swerve.resetEncoders();
    // container.poseEstimator.resetPose();
    // container.auto.executeRoutine();
  }

  @Override public void autonomousPeriodic() {

  }

  @Override public void disabledPeriodic() {
    container.led.rainbow();
    // container.led.shockwave();
    // container.led.setStaticColor(new Color(255, 0, 0));
  }

  @Override public void teleopInit() {
    container.swerve.setDefaultCommand(new SwerveDriveCommand(container.driverController, container.swerve));
  }

  @Override public void teleopPeriodic() {

  }
}
