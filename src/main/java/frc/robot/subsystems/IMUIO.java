package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

/** The IO layer for an IMU. Moving to use quest instead. */
@Deprecated(forRemoval = true, since = "20250501")
public interface IMUIO {
  /** The inputs from the IMU. Access using <code>IMUInputsAutoLogged</code>. */
  @AutoLog
  class IMUIOInputs {
    /** The rotation from the gyro */
    public Rotation3d rotation = new Rotation3d();
    /** The acceleration in the x-axis in meters per second squared */
    public double xAccelMPS = 0.0;
    /** The acceleration in the y-axis in meters per second squared */
    public double yAccelMPS = 0.0;
    /** The acceleration in the z-axis in meters per second squared */
    public double zAccelMPS = 0.0;
    /** The temperature from the IMU */
    public double tempC = 0.0;
  }

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs to update
   */
  default void updateInputs(IMUIOInputs inputs) {}
}
