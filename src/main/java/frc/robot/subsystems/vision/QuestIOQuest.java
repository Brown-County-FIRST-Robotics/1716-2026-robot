package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.Arrays;

public class QuestIOQuest implements QuestIO {
  QuestNav qn;

  public QuestIOQuest(QuestNav qn) {
    this.qn = qn;
  }

  @Override
  public void updateInputs(QuestIOInputs inputs) {
    var frams = qn.getAllUnreadPoseFrames();
    if (frams.length > 0) {
      inputs.whereami = frams[0].questPose3d();
    }
    inputs.raw_poses = (Pose3d[]) Arrays.stream(frams).map(PoseFrame::questPose3d).toArray();
  }
}
