package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurConstants;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  RollersIO io;
  RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private final Debouncer beltsConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer starsConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer kickerConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Alert beltsDisconnectedAlert;
  private final Alert starsDisconnectedAlert;
  private final Alert kickerDisconnectedAlert;

  public Rollers(RollersIO io) {
    this.io = io;
    starsDisconnectedAlert = new Alert("Star wheel motor disconnected", Alert.AlertType.kError);
    beltsDisconnectedAlert = new Alert("Belt motor disconnected", Alert.AlertType.kError);
    kickerDisconnectedAlert = new Alert("Kicker motor disconnected", Alert.AlertType.kError);
  }

  public void setSpeeds(double beltsVelocity, double starsVelocity, double kicker_velocity) {
    io.commandSpeed(beltsVelocity, starsVelocity, kicker_velocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("rollers", inputs);
    beltsDisconnectedAlert.set(!beltsConnectedDebouncer.calculate(inputs.beltsConnected));
    starsDisconnectedAlert.set(!starsConnectedDebouncer.calculate(inputs.starsConnected));
    kickerDisconnectedAlert.set(!kickerConnectedDebouncer.calculate(inputs.kickerConnected));
  }
}
