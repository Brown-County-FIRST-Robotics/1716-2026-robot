package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;

public class VisionSLAMIOQuest implements VisionSLAMIO {
  final NetworkTable table = NetworkTableInstance.getDefault().getTable("questnav");
  final DoubleArraySubscriber questRot =
      table.getDoubleArrayTopic("quaternion").subscribe(new double[] {});
  final DoubleArraySubscriber questPosition =
      table.getDoubleArrayTopic("position").subscribe(new double[] {});
  final IntegerSubscriber questFrames = table.getIntegerTopic("frameCount").subscribe(0);
  final DoubleSubscriber battery = table.getDoubleTopic("batteryPercent").subscribe(0);
  private double lastProcessedHeartbeatId = 0;
  private final DoubleSubscriber heartbeatRequestSub =
      table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
  private final DoublePublisher heartbeatResponsePub =
      table.getDoubleTopic("heartbeat/robot_to_quest").publish();

  @Override
  public void updateInputs(VisionSLAMIOInputs inputs) {
    double[] rawRotation = questRot.get();
    double[] rawTranslation = questPosition.get();
    if (rawRotation.length > 0 && rawTranslation.length > 0) {
      inputs.present = true;
      inputs.questTranslation =
          new Translation3d(rawTranslation[0], rawTranslation[1], rawTranslation[2]);
      inputs.questQuat =
          new Quaternion(rawRotation[0], rawRotation[1], rawRotation[2], rawRotation[3]);
      inputs.frames = questFrames.get();
    } else {
      inputs.present = false;
    }
    inputs.battPercent = battery.get();
    processHeartbeat();
  }

  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }
}
