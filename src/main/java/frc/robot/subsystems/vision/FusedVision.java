package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.utils.*;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** The vision subsystem */
public class FusedVision extends PeriodicRunnable {
  final VisionIO io;
  public final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
  final SwerveDrivetrain drivetrain;

  final VisionSLAMIO slamio;
  final VisionSLAMIOInputsAutoLogged slamInputs = new VisionSLAMIOInputsAutoLogged();
  final Transform3d slamPose;
  Pose3d zero = Pose3d.kZero;
  Pose3d lastHeadsetPoseSeen = Pose3d.kZero;
  boolean seen = false;

  public void setpos(Pose2d pos) {
    zero = lastHeadsetPoseSeen;
    zero = zero.plus(new Transform3d(Pose3d.kZero, new Pose3d(pos)).inverse());
  }

  public FusedVision(
      SwerveDrivetrain drivetrain, Transform3d slamPose, VisionSLAMIO slamio, VisionIO visionIO) {
    this.drivetrain = drivetrain;
    this.slamPose = slamPose;
    this.slamio = slamio;
    drivetrain.getPE().pt = Optional.of(this);
    this.io = visionIO;
    new CustomAlerts.CustomAlert(
        Alert.AlertType.WARNING,
        () -> slamInputs.battPercent < 20.0,
        () -> "Quest battery is at " + slamInputs.battPercent + "%");
  }

  public Pose2d getSlamPose() {
    return Pose3d.kZero.plus(lastHeadsetPoseSeen.minus(zero)).toPose2d();
  }

  long lastFrame = 0;

  long frameTimeDelta = 0;

  public boolean isActive() {
    return frameTimeDelta > 0;
  }

  @Override
  public void periodic() {
    slamio.updateInputs(slamInputs);
    Logger.processInputs("Vision/SLAM", slamInputs);

    frameTimeDelta = slamInputs.frames - lastFrame;
    lastFrame = slamInputs.frames;
    Logger.recordOutput("Vision/QuestConnected", isActive());

    if (slamInputs.present) {
      var questRotVec = slamInputs.questQuat.toRotationVector(); // In quest coordinate system
      var headsetRotation =
          new Rotation3d(
              VecBuilder.fill(
                  questRotVec.get(0),
                  -questRotVec.get(2),
                  -questRotVec.get(1))); // In WPILIB coordinates
      var headsetPos =
          new Pose3d(
              -slamInputs.questTranslation.getZ(),
              -slamInputs.questTranslation.getX(),
              -slamInputs.questTranslation.getY(),
              headsetRotation);
      var robotHeadsetPos = headsetPos.plus(slamPose.inverse());

      if (!seen) {
        seen = true;
        lastHeadsetPoseSeen = robotHeadsetPos;
        zero = robotHeadsetPos;
      }

      Logger.recordOutput("asdf", Pose3d.kZero.plus(robotHeadsetPos.minus(zero)));

      lastHeadsetPoseSeen = robotHeadsetPos;
    }
    io.updateInputs(inputs);
    Logger.processInputs("Vision/AprilInputs", inputs);
  }
}
