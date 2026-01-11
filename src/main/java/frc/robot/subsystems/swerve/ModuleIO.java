package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

/** The IO layer for a swerve module */
public interface ModuleIO {
  /** The inputs to the code from a swerve module */
  @AutoLog
  class ModuleIOInputs {
    /** The velocity according to the encoders */
    public double thrustVel = 0.0;
    /** The thrust encoder position */
    public double thrustPos = 0.0;
    /** The angle of the steering according to the relative encoder in rotations */
    public double relativeSensorAngle = 0.0;
    /** The angular velocity of the steer motor according to the relative encoder */
    public double relativeSensorOmega = 0.0;
    /** The output of the thrust motor */
    public double thrustOutput = 0.0;
    /** The angular velocity of the steer motor according to the encoder on the wheel shaft */
    public double absSensorOmega = 0.0;
    /** The angle of the steering motor according to the encoder on the wheel shaft */
    public double absSensorAngle = 0.0;
    /** The temperature of the steer motor in Celsius */
    public double steerTempC = 0.0;
    /** The temperature of the thrust motor in Celsius */
    public double thrustTempC = 0.0;
    /** The closed-loop error of the thrust motor */
    public double thrustErr = 0.0;
    /** The swerve module offset */
    public double offset = 0.0;
  }
  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Commands a steer angle and thrust speed to the module
   *
   * @param ang The command angle, as relative encoder position
   * @param vel The thrust velocity (meters per second)
   */
  default void setCmdState(double ang, double vel) {}
}
