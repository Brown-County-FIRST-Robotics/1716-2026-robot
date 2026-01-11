package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.FusedVision;
import java.util.Optional;

// TODO:Re-evalulate this file

/** A pose estimator that fuses vision and odometry updates */
public class PoseEstimator {
  // Ship of Theseused from
  public Optional<FusedVision> pt = Optional.empty();
  // https://github.com/wpilibsuite/allwpilib/blob/1db3936965bd8ed33224ad388cf9f16d12999a08/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java
  Pose2d current = Pose2d.kZero;
  boolean usedVis = false;

  public Pose2d getPose() {
    return current;
  }

  public void setDelta(double delta) {
    current =
        new Pose2d(
            current.getTranslation(), new Rotation2d(current.getRotation().getRadians() + delta));
  }

  public void setPose(Pose2d pz) {
    current = pz;
  }

  public void feed() {
    if (new XboxController(0).getXButton()) {
      Pose2d face = FieldConstants.getFace(0);
      Pose2d plus =
          face.plus(new Transform2d(0, -19.0 * 0.0254, new Rotation2d()))
              .plus(new Transform2d(16.0 * 0.0254, 16.0 * 0.0254, Rotation2d.kZero));
      setPose(plus);
    }
    if (new XboxController(0).getYButton()) {
      Pose2d face = FieldConstants.flip(FieldConstants.getFace(0));
      Pose2d plus =
          face.plus(new Transform2d(0, -19.0 * 0.0254, new Rotation2d()))
              .plus(new Transform2d(16.0 * 0.0254, 16.0 * 0.0254, Rotation2d.kZero));
      setPose(plus);
    }
    if (new XboxController(0).getAButton()) {
      usedVis = false;
    }
    if (pt.isPresent()) {
      if (!usedVis && pt.get().isActive()) {
        if (pt.get().inputs.pose.isPresent()) {
          pt.get().setpos(pt.get().inputs.pose.get().toPose2d());
          usedVis = true;
        }
      }
    }
  }

  /** Constructs a new pose estimator */
  public PoseEstimator() {
    new PeriodicRunnable() {
      @Override
      public void periodic() {
        feed();
      }
    };
  }
}
