package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import frc.robot.utils.PeriodicRunnable;

public class TurretCamera extends PeriodicRunnable {

  VisionIO visionIO;
  VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public TurretCamera(VisionIO visionIo) {
    super();
    this.visionIO = visionIo;
  }

  @Override
  public void periodic() {

    visionIO.updateInputs(inputs);
    Logger.processInputs("turretCamera", inputs);
  }
}
