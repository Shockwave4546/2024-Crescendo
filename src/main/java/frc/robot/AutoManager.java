package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoManager {
  /**
   * Automatically generates all the autonomous modes from ../pathplanner/autos
   */
  private final SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser("Do nothing.");

  /**
   * Registers all Commands into NamedCommands.
   */
  public AutoManager() {
    Constants.Tabs.MATCH.add("Autonomous", chooser).withSize(3, 2);
  }

  /**
   * Schedules the selected autonomous mode.
   */
  public void executeRoutine() {
    chooser.getSelected().schedule();
  }
}