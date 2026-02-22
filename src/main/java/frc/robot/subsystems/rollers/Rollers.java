package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurConstants;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  RollersIO io;
  RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private final Debouncer rollerConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer kickerConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Alert rollerDisconnectedAlert;
  private final Alert kickerDisconnectedAlert;

  public Rollers(RollersIO io) {
    this.io = io;
    rollerDisconnectedAlert =
        new Alert(
            "Roller motor disconnected"
                + (DriverStation.isFMSAttached() ? ". Good fucking luck. " : ""),
            Alert.AlertType.kError);
    kickerDisconnectedAlert =
        new Alert(
            "Kicker motor disconnected"
                + (DriverStation.isFMSAttached() ? ". Good fucking luck. " : ""),
            Alert.AlertType.kError);
  }

  public void setSpeeds(double rollerVelocity, double kicker_velocity) {
    io.commandSpeed(rollerVelocity, kicker_velocity);
  }

  public void jset(double volts) {
    io.justSet(volts);
  }

  public Command spinRollers() {
    return Commands.run(() -> setSpeeds(20, 20), this);
  }

  public Command reverseRollers() {
    return Commands.run(() -> setSpeeds(-20, -20), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("rollers", inputs);
    kickerDisconnectedAlert.set(kickerConnectedDebouncer.calculate(inputs.kickerConnected));
    rollerDisconnectedAlert.set(rollerConnectedDebouncer.calculate(inputs.rollersConnected));
  }
}
