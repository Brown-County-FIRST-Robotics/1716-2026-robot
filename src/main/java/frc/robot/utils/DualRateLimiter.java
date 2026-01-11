package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;

/**
 * Like {@link edu.wpi.first.math.filter.SlewRateLimiter SlewRateLimiter}, but uses a different rate
 * for acceleration and deceleration
 */
public class DualRateLimiter {
  private final double accelRate;
  private final double deccelRate;
  private double prevVal;
  private double prevTime;

  /**
   * Makes a new rate limiter
   *
   * @param accelRate The maximum acceleration
   * @param deccelRate The maximum deceleration
   * @param prevVal The previous value of the limiter
   */
  public DualRateLimiter(double accelRate, double deccelRate, double prevVal) {
    this.accelRate = accelRate;
    this.deccelRate = deccelRate;
    this.prevVal = prevVal;
    this.prevTime = MathSharedStore.getTimestamp();
  }

  private static double clamp(double v, double mn, double mx) {
    return Math.min(mx, Math.max(v, mn));
  }

  /**
   * Constructs the rate limiter with an initial value of 0
   *
   * @param accelRate The maximum acceleration
   * @param deccelRate The maximum deceleration
   */
  public DualRateLimiter(double accelRate, double deccelRate) {
    this(accelRate, deccelRate, 0);
  }

  /**
   * Calculates the limited value
   *
   * @param val The value to limit
   * @return The limited value
   */
  public double calculate(double val) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        clamp(
                (val - prevVal) * Math.signum(prevVal == 0 ? Double.MIN_VALUE : prevVal),
                -deccelRate * elapsedTime,
                accelRate * elapsedTime)
            * Math.signum(prevVal == 0 ? Double.MIN_VALUE : prevVal);
    prevTime = currentTime;
    return prevVal;
  }

  /**
   * Sets the value, ignoring limits
   *
   * @param val The new value to use
   */
  public void reset(double val) {
    prevTime = MathSharedStore.getTimestamp();
    prevVal = val;
  }
}
