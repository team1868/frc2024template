package frc.robot.constants.enums;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Mostly for PathPlanner, will need to be refactored for
public class AutonomousAction {
  private final String name;
  private final Command underlying;

  private Command command;

  private static final List<AutonomousAction> actionList = new ArrayList<AutonomousAction>();
  private static final Map<String, Command> eventMap = new HashMap<String, Command>();

  AutonomousAction(String name, Command underlying) {
    this.name = name;
    this.underlying = underlying;
  }

  private static void register(AutonomousAction action) {
    actionList.add(action);
    eventMap.put(action.name, action.command);
  }

  public static void registerAutonomousAction(String name, double waitSeconds) {
    AutonomousAction action = new AutonomousAction(name, new WaitCommand(waitSeconds));
    action.command = action.underlying;
    register(action);
  }

  // public static void registerAutonomousAction(String name, LedColors color) {
  //   AutonomousAction action = new AutonomousAction(
  //       name, new InstantCommand(() -> controller.setSolidColor(color, LedSections.ALL))
  //   );
  //   action.command = action.underlying;
  //   register(action);
  // }

  public static void registerAutonomousAction(String name, String message) {
    AutonomousAction action = new AutonomousAction(name, new PrintCommand(message));
    // unclear if we can just skip this step or not
    action.command = new InstantCommand(() -> action.underlying.schedule());
    register(action);
  }

  public static void registerAutonomousAction(String name, Command command) {
    AutonomousAction action = new AutonomousAction(name, command);
    action.command = new InstantCommand(() -> action.underlying.schedule());
    register(action);
  }

  public static Map<String, Command> getEventMap() {
    return eventMap;
  }
}
