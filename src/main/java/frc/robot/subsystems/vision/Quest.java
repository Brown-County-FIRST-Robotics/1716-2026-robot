package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Quest implements Subsystem {
  QuestIO io;
  QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Quest",inputs);
  }
}
