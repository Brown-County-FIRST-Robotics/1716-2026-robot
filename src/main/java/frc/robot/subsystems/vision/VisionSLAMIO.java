package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionSLAMIO {
  @AutoLog
  class VisionSLAMIOInputs {
    Quaternion questQuat = new Quaternion();
    Translation3d questTranslation = Translation3d.kZero;
    double battPercent = 0.0;
    long frames = 0;
    double timestamp = 0.0;
    boolean present = false;
  }

  default void updateInputs(VisionSLAMIOInputs inputs) {}
}
