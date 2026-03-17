package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import frc.robot.FieldConstants;
import frc.robot.utils.PeriodicRunnable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Quest extends PeriodicRunnable {
  QuestIO io;
  QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();
  private static final Transform3d posOnRobot =
      new Transform3d(
          new Translation3d(-10.59 * 0.0254, -12.37 * 0.0254, 0.5),
          new Rotation3d(0, 0, 200.0 * Math.PI / 180.0));
  private Pose3d realPose = new Pose3d(FieldConstants.ip());
  private Pose3d rqp = null;

  public Quest(QuestIO io) {
    super();
    this.io = io;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public Rotation2d gyroLikeYaw() {
    return realPose.getRotation().toRotation2d();
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return realPose.toPose2d();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Quest", inputs);
    var nqp = inputs.whereami.plus(posOnRobot.inverse());
    if (rqp == null) {
      rqp = nqp;
    }
    var d = nqp.minus(rqp);
    rqp = nqp;
    realPose = realPose.plus(d);
  }
}
