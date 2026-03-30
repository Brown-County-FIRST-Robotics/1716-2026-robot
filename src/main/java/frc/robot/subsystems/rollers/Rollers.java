package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurConstants;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  RollersIO io;
  RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private final Debouncer kicklerConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer rollerConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer kickerConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Alert kicklerDisconnectedAlert;
  private final Alert rollerDisconnectedAlert;
  private final Alert kickerDisconnectedAlert;

  public Rollers(RollersIO io) {
    this.io = io;
    rollerDisconnectedAlert = new Alert("TICKLER motor disconnected", Alert.AlertType.kError);
    kicklerDisconnectedAlert = new Alert("Kickler motor disconnected", Alert.AlertType.kError);
    kickerDisconnectedAlert = new Alert("Kicker motor disconnected", Alert.AlertType.kError);
  }

  public void setSpeeds(double kicklerVelocity, double rollerVelocity, double kicker_velocity) {
    io.commandSpeed(kicklerVelocity, rollerVelocity, kicker_velocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("rollers", inputs);
    kicklerDisconnectedAlert.set(!kicklerConnectedDebouncer.calculate(inputs.kicklerConnected));
    rollerDisconnectedAlert.set(!rollerConnectedDebouncer.calculate(inputs.rollersConnected));
    kickerDisconnectedAlert.set(!kickerConnectedDebouncer.calculate(inputs.kickerConnected));
  }
}
