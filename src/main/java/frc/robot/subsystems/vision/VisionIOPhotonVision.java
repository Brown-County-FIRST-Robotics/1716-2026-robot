package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The IO layer for one PhotonVision camera */
public class VisionIOPhotonVision implements VisionIO {

  PhotonCamera turretCamera;

  public VisionIOPhotonVision(String cameraName) {

    turretCamera = new PhotonCamera("turretCamera");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    PhotonPipelineResult results = turretCamera.getLatestResult();
    boolean hasTargets = results.hasTargets();
    List<PhotonTrackedTarget> targets = results.getTargets();

    inputs.yaw = new double[targets.size()];
    inputs.pitch = new double[targets.size()];
    inputs.roll = new double[targets.size()];
    inputs.area = new double[targets.size()];
    inputs.idNumber = new int[targets.size()];

    for (int i = 0; i < targets.size(); i++) {
      var target = targets.get(0);
      inputs.yaw[i] = target.getYaw();
      inputs.pitch[i] = target.getPitch();
      inputs.area[i] = target.getArea();
      inputs.idNumber[i] = target.getFiducialId();
    }
  }
}
