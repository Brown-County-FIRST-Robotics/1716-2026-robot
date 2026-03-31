package frc.robot.subsystems.vision;

import gg.questnav.questnav.QuestNav;

public class QuestIOQuest implements QuestIO {
  QuestNav qn;

  public QuestIOQuest(QuestNav qn) {
    this.qn = qn;
  }

  @Override
  public void updateInputs(QuestIOInputs inputs) {
    qn.commandPeriodic();
    var frames = qn.getAllUnreadPoseFrames();
    if (frames.length > 0) {
      inputs.lastPose = frames[frames.length - 1].questPose3d();
    }
    inputs.connected = qn.isConnected();
    inputs.tracking = qn.isTracking();
    var bat = qn.getBatteryPercent();
    inputs.battery = bat.isPresent() ? bat.getAsInt() : -1;
  }
}
