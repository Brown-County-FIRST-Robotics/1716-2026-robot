package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera extends SubsystemBase {
  @Override
  public void periodic() {
    PhotonCamera camera = new PhotonCamera("turretCamera");
    var collin = camera.getLatestResult();
    boolean hasTargets = collin.hasTargets();
    List<PhotonTrackedTarget> targets = collin.getTargets();

    var target = targets.get(0);
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();

    int targetID = target.getDetectedObjectClassID();

    Logger.recordOutput("yaw", yaw);
    Logger.recordOutput("pitch", pitch);
    Logger.recordOutput("area", area);
    Logger.recordOutput("targetID", targetID);
  }
}
