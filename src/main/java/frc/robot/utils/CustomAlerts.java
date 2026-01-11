package frc.robot.utils;

import com.studica.frc.AHRS;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Utility class for some custom alerts */
public class CustomAlerts {
  /** Alert that takes a supplier, and calls it once every tick */
  public static class CustomAlert extends PeriodicRunnable {
    final Alert alert;
    final BooleanSupplier isActive;
    final Supplier<String> message;

    /**
     * Makes a new CustomAlert
     *
     * @param level The alert level
     * @param isActive Whether the alert should be displayed
     * @param message The message to show when displayed
     */
    public CustomAlert(Alert.AlertType level, BooleanSupplier isActive, Supplier<String> message) {
      super();
      this.isActive = isActive;
      this.message = message;
      alert = new Alert(message.get(), level);
    }

    /**
     * Makes a new CustomAlert
     *
     * @param level The alert level
     * @param isActive Whether the alert should be displayed
     * @param message The message to show when displayed
     */
    public CustomAlert(Alert.AlertType level, BooleanSupplier isActive, String message) {
      this(level, isActive, () -> message);
    }

    @Override
    public void periodic() {
      if (isActive.getAsBoolean()) {
        alert.setText(message.get());
        alert.set(true);
      } else {
        alert.set(false);
      }
    }
  }

  /** An alert that will activate if latch is called, the stay activated for a given duration */
  public static class TimeLatchAlert extends PeriodicRunnable {
    final Timer lastTrue = new Timer();
    boolean hasTrue = false;
    double duration;
    final Alert alert;

    /**
     * Makes a new TimeLatchAlert
     *
     * @param level The alert level
     * @param duration The duration the alert will stay active for
     * @param message The message to show
     */
    public TimeLatchAlert(Alert.AlertType level, double duration, String message) {
      super();
      alert = new Alert(message, level);
    }

    /** Activates the alert for a given time */
    public void latch() {
      hasTrue = true;
      lastTrue.restart();
    }

    @Override
    public void periodic() {
      alert.set(hasTrue);
      if (lastTrue.hasElapsed(duration) && hasTrue) {
        hasTrue = false;
      }
    }
  }

  /** An alert that will activate if feed has not been called for a given time */
  public static class TimeoutAlert extends PeriodicRunnable {
    final double timeout;
    final Alert alert;
    double lastFeedTime;

    /**
     * Makes a new TimeoutAlert
     *
     * @param level The alert level
     * @param timeout The maximum time between feeds
     * @param message The message to show when activated
     */
    public TimeoutAlert(Alert.AlertType level, double timeout, String message) {
      super();
      this.timeout = timeout;
      lastFeedTime = Timer.getFPGATimestamp();
      alert = new Alert(message, level);
    }

    /** Resets the watchdog timer and deactivates the alert */
    public void feed() {
      lastFeedTime = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
      alert.set(Timer.getFPGATimestamp() > lastFeedTime + timeout);
    }
  }

  public static void makeOverTempAlert(
      DoubleSupplier tempSupplier, double errTemp, String deviceName) {
    new CustomAlert(
        Alert.AlertType.ERROR,
        () -> tempSupplier.getAsDouble() >= errTemp,
        () ->
            deviceName
                + " is currently "
                + tempSupplier.getAsDouble()
                + " degrees celsius (max "
                + errTemp
                + "). Prolonged usage could permanently damage the motor");
  }

  /**
   * Makes a `CustomAlert`(An alert that activates if the given function is true) that will be a
   * warning over a certain temperature, and an error above a second temperature
   *
   * @param tempSupplier Provides the temperature in Celsius
   * @param errTemp The minimum temperature that will cause the alert to be an error
   * @param warnTemp The minimum temperature that will cause the alert to be a warning
   * @param deviceName The name of the device this alert is for
   */
  public static void makeOverTempAlert(
      DoubleSupplier tempSupplier, double errTemp, double warnTemp, String deviceName) {
    makeOverTempAlert(tempSupplier, errTemp, deviceName);
    new CustomAlert(
        Alert.AlertType.WARNING,
        () -> (tempSupplier.getAsDouble() >= warnTemp && tempSupplier.getAsDouble() < errTemp),
        () ->
            deviceName
                + " is currently "
                + tempSupplier.getAsDouble()
                + " degrees celsius (max "
                + errTemp
                + ")");
  }

  /**
   * Makes Alerts for when CAN utilization is too high
   *
   * @param errUtilization The minimum utilization to trigger an alert
   */
  public static void makeCANFailAlerts(double errUtilization) {
    var stat = new CANStatus();
    CANJNI.getCANStatus(stat);

    new CustomAlert(
        Alert.AlertType.ERROR,
        () -> errUtilization < stat.percentBusUtilization,
        () ->
            "CAN Bus utilization is "
                + stat.percentBusUtilization
                + "(max "
                + errUtilization
                + "). All mechanisms may soon cease to function. ");
  }

  /**
   * Makes alerts for when the NAVX is unplugged or calibrating
   *
   * @param navx The NAVX to track
   */
  public static void makeNavxFailAlerts(AHRS navx) {
    new CustomAlert(
        Alert.AlertType.ERROR,
        () -> !navx.isConnected(),
        "Navx is not connected. This will cause pose estimation and field-oriented driving to fail. ");
    new CustomAlert(
        Alert.AlertType.WARNING,
        navx::isCalibrating,
        "Navx is calibrating. Please do not move the robot. ");
  }
}
