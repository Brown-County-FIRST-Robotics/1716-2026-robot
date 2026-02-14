package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** The IO layer for one camera */
public interface VisionIO {
  /** The inputs from a camera */
  @AutoLog
  class VisionIOInputs {

    public int[] idNumber = new int[] {};
    public double[] yaw = new double[] {};
    public double[] pitch = new double[] {};
    public double[] roll = new double[] {};
    public double[] area = new double[] {};
  }

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(VisionIOInputs inputs) {}
}
