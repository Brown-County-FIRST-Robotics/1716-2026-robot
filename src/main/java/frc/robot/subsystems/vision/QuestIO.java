package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface QuestIO {
  @AutoLog
  class QuestIOInputs {
    Pose3d lastPose = new Pose3d();
    boolean connected = false;
    boolean tracking = false;
  }

  default void updateInputs(QuestIOInputs inputs) {}
}
