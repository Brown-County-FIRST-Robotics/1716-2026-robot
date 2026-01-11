package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** The IO layer for one camera */
public interface VisionIO {
  /** The inputs from a camera */
  class VisionIOInputs implements LoggableInputs {
    public Optional<Pose3d> pose = Optional.empty();
    public Optional<Double> timestamp = Optional.empty();

    @Override
    public void toLog(LogTable table) {
      table.put("hasPose", pose.isPresent());
      table.put("hasTimestamp", timestamp.isPresent());
      table.put("Pose", pose.orElse(new Pose3d()));
      table.put("Timestamp", timestamp.orElse(0.0));
    }

    @Override
    public void fromLog(LogTable table) {
      pose =
          table.get("hasPose", false)
              ? Optional.of(table.get("Pose", new Pose3d()))
              : Optional.empty();
      timestamp =
          table.get("hasTimestamp", false)
              ? Optional.of(table.get("Timestamp", 0.0))
              : Optional.empty();
    }
  }

  /**
   * Updates the inputs
   *
   * @param inputs A reference to the inputs
   */
  default void updateInputs(VisionIOInputs inputs) {}
}
