package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() { throw new UnsupportedOperationException("Stop."); }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
