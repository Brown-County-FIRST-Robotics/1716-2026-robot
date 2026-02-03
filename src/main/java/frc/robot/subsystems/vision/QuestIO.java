package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO {
  @AutoLog
  class QuestIOInputs {
    Pose3d whereami = new Pose3d();
    boolean connected = false;
    Pose3d[] raw_poses = new Pose3d[] {};
    double[] raw_timestamps = new double[] {};
  }

  default void updateInputs(QuestIOInputs inputs) {}
}
