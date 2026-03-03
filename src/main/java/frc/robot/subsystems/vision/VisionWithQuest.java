package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.FieldConstants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.PeriodicRunnable;
import java.util.Optional;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class VisionWithQuest extends PeriodicRunnable {
  AprilTagDetector detector = new AprilTagDetector();
  HttpCamera camera;

  CvSink cvSink;

  Drive drive;
  LEDs leds;

  public VisionWithQuest(Drive drive, LEDs leds) {
    super();
    detector.addFamily("tag36h11");
    this.drive = drive;

    camera = new HttpCamera("QuestNav", "http://10.17.17.225:5801/video", HttpCamera.HttpCameraKind.kMJPGStreamer);
    CameraServer.startAutomaticCapture(camera);
    cvSink = CameraServer.getVideo();

    if (leds != null) {
      leds.setMode(LEDMode.SOLID);
      leds.setColor(Color.kRed);
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      Mat mat = new Mat();
      Mat greyscale = new Mat();

      var poseEstConfig =
          new AprilTagPoseEstimator.Config(
              0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
      var estimator = new AprilTagPoseEstimator(poseEstConfig);

      if (cvSink.grabFrame(mat) == 0) {
        System.out.println("Frame grab failed");
        return;
      }

      Imgproc.cvtColor(mat, greyscale, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(greyscale);

      // Accumulates the relative positions before avg
      double sumX = 0;
      double sumY = 0;
      double sumRotZ = 0;

      int numGoodTags = 0;

      System.out.println("Found " + detections.length + " POIs");

      for (AprilTagDetection detection : detections) {
        final int id = detection.getId();

        // Get coordinates of the tag relative to the robot
        Transform3d pose = estimator.estimate(detection);
        final double tagX = pose.getX();
        final double tagY = pose.getY();
        final double tagRotZ = pose.getRotation().getZ();

        System.out.println("FOUND APRIL TAG " + id + " AT (" + tagX + ", " + tagY + ")");

        // Get the absolute coordinates of the tag
        Optional<Pose3d> exp = FieldConstants.layout.getTagPose(id);
        if (exp.isEmpty()) continue;
        Pose3d expectedPos = exp.get();

        final double x = expectedPos.getX();
        final double y = expectedPos.getY();

        final double rotZ = expectedPos.getRotation().getZ();

        // Accumulate diffs
        sumX += tagX - x;
        sumY += tagY - y;

        sumRotZ += tagRotZ - rotZ;

        numGoodTags++;
      }

      if (numGoodTags > 0) {
        var pos = new Translation2d(sumX / numGoodTags, sumY / numGoodTags);
        var rot = new Rotation2d(sumRotZ / numGoodTags);
        drive.setPose(new Pose2d(pos, rot));

        if (leds != null) leds.setColor(Color.kGreen);
      }
    }
  }
}
