package org.dovershockwave;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.dovershockwave.swerve.commands.SwerveDriveCommand;

public class Robot extends TimedRobot {
  private RobotContainer container;

  @Override public void robotInit() {
    container = new RobotContainer();
    container.swerve.zeroGyro();
    container.swerve.resetEncoders();
    container.poseEstimator.resetPose();
    container.amp.resetPosition();
  }

  @Override public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override public void autonomousInit() {
    CommandScheduler.getInstance().removeDefaultCommand(container.swerve);
    container.swerve.zeroGyro();
    container.swerve.resetEncoders();
    container.poseEstimator.resetPose();
    try {
      Thread.sleep(500);
    } catch (InterruptedException ignored) {
      
    }
    container.auto.executeRoutine();
  }

  @Override public void autonomousPeriodic() {

  }

  @Override public void disabledPeriodic() {
     container.led.rainbow();
  }

  @Override public void teleopInit() {
    container.swerve.setDefaultCommand(new SwerveDriveCommand(container.driverController, container.swerve));
  }

  @Override public void teleopPeriodic() {
   
  }
}