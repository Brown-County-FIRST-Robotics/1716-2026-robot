package frc.robot.utils;

import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** A dashboard number that can be used for tunable values */
public class LoggedTunableNumber extends PeriodicRunnable {
  private static final String tableKey = "Tuning";
  private final String key;
  private final String name;
  private LoggedNetworkNumber dashboardNumber;
  private double defaultValue;
  private boolean hasDefault;
  private double lastHasChangedValue;
  private double lastPeriodicValue;
  private Consumer<Double> handler = (Double v) -> {};
  private Alert changedAlert;

  /**
   * Makes a new tunable
   *
   * @param name The name of the number on the dashboard
   */
  public LoggedTunableNumber(String name) {
    super();
    this.name = name;
    key = "Shuffleboard/" + tableKey + "/" + name;
  }

  /**
   * Makes a new tunable with a default value
   *
   * @param name The name of the number on the dashboard
   * @param defaultValue The default value
   */
  public LoggedTunableNumber(String name, double defaultValue) {
    this(name);
    initDefault(defaultValue);
  }

  /**
   * Initializes the dashboard input
   *
   * @param defaultValue The default value to publish to the dashboard
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      changedAlert =
          new Alert(
              "Tunable number \""
                  + name
                  + "\" is not at its default ("
                  + defaultValue
                  + "). This could impair robot functions. ",
              Alert.AlertType.WARNING);
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  /**
   * Gets the current value of the number
   *
   * @return The value
   */
  public double get() {
    return hasDefault ? dashboardNumber.get() : 0.0;
  }

  /**
   * Checks if it has changed since this method was last called
   *
   * @return If it has changed
   */
  public boolean hasChanged() {
    double currentVal = get();
    if (currentVal != lastHasChangedValue) {
      lastHasChangedValue = currentVal;
      return true;
    }
    return false;
  }

  /**
   * Adds a handler for changes
   *
   * @param changeHandler The handler to use
   */
  public void attach(Consumer<Double> changeHandler) {
    handler = changeHandler;
    if (hasDefault) {
      handler.accept(get());
    }
  }

  @Override
  public void periodic() {
    double currentVal = get();
    if (currentVal != lastPeriodicValue) {
      lastPeriodicValue = currentVal;
      handler.accept(currentVal);
    }
    changedAlert.set(currentVal != defaultValue);
  }
}
