package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Quest implements Subsystem {
  QuestIO io;
  QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();
  private static final Transform3d posOnRobot =
      new Transform3d(new Translation3d(-0.2, -0.25, 0.5), new Rotation3d(0, 0, Math.PI * 1.1));
  private Pose3d realPose = new Pose3d();

  public Quest(QuestIO io) {
    this.io = io;
  }

  public boolean connected() {
    return inputs.connected;
  }

  public Rotation2d gyroLikeYaw() {
    return realPose.getRotation().toRotation2d();
  }

  public Pose3d getPose() {
    return realPose;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Quest", inputs);
    realPose = inputs.whereami.plus(posOnRobot.inverse());
  }
}
