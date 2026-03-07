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
    var frams = qn.getAllUnreadPoseFrames();
    if (frams.length > 0) {
      inputs.whereami = frams[0].questPose3d();
    }
    inputs.connected = qn.isConnected();
    // inputs.raw_poses = (Pose3d[]) Arrays.stream(frams).map(PoseFrame::questPose3d).toArray();
  }
}
