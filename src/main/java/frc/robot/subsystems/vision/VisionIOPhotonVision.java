package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** The IO layer for one PhotonVision camera */
public class VisionIOPhotonVision implements VisionIO {
  final PhotonCamera cam;

  final PhotonPoseEstimator photonPoseEstimator;
  /**
   * Constructs a new <code>VisionIOPhotonVision</code> from a camera name and pose
   *
   * @param cam_name The name of the camera
   */
  public VisionIOPhotonVision(String cam_name, Transform3d camPose) {
    cam = new PhotonCamera(cam_name);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camPose);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    var urres = cam.getAllUnreadResults();
    if (!urres.isEmpty()) {
      var k = photonPoseEstimator.update(urres.get(0));
      if (k.isPresent()) {
        inputs.pose = Optional.ofNullable(k.get().estimatedPose);
        inputs.timestamp = Optional.of(k.get().timestampSeconds);
      } else {
        inputs.pose = Optional.empty();
        inputs.timestamp = Optional.empty();
      }

    } else {
      inputs.pose = Optional.empty();
      inputs.timestamp = Optional.empty();
    }
  }
}
