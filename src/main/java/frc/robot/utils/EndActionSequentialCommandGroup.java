package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;


public class EndActionSequentialCommandGroup extends Command {
  private final List<Command> commands = new ArrayList<>();
  private final Command endAction;
  private int currentCommandIndex = -1;
  private boolean runWhenDisabled = true;
  private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

  public EndActionSequentialCommandGroup(Command endAction) {
    this.endAction = endAction;
  }

  public final void addCommands(Command... commands) {
    if (currentCommandIndex != -1) {
      throw new IllegalStateException("Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (final var command : commands) {
      this.commands.add(command);
      m_requirements.addAll(command.getRequirements());
      runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }


  @Override public final void initialize() {
    currentCommandIndex = 0;

    if (!commands.isEmpty()) {
      commands.get(0).initialize();
    }
  }

  @Override public final void execute() {
    if (commands.isEmpty()) {
      return;
    }

    final var currentCommand = commands.get(currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      currentCommandIndex++;
      if (currentCommandIndex < commands.size()) {
        commands.get(currentCommandIndex).initialize();
      }
    }
  }

  @Override public final void end(boolean interrupted) {
    if (interrupted && !commands.isEmpty() && currentCommandIndex > -1 && currentCommandIndex < commands.size()) {
      commands.get(currentCommandIndex).end(true);
    }

    endAction.schedule();
    currentCommandIndex = -1;
  }

  @Override public final boolean isFinished() {
    return currentCommandIndex == commands.size();
  }

  @Override public boolean runsWhenDisabled() {
    return runWhenDisabled;
  }

  @Override public InterruptionBehavior getInterruptionBehavior() {
    return interruptBehavior;
  }
}