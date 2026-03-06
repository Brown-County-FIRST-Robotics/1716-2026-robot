package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  IntakeIO io;
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Debouncer intakeConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer extendConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Alert intakeDisconnectedAlert;
  private final Alert extendDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    intakeDisconnectedAlert =
        new Alert(
            "Intake motor disconnected"
                + (DriverStation.isFMSAttached() ? ". All hope is lost. " : ""),
            Alert.AlertType.kError);
    extendDisconnectedAlert =
        new Alert(
            "Intake extension motor disconnected"
                + (DriverStation.isFMSAttached()
                    ? " ___  _   _  _ ___ ___ \n"
                        + "| _ \\/_\\ | \\| |_ _/ __|\n"
                        + "|  _/ _ \\| .` || | (__ \n"
                        + "|_|/_/ \\_\\_|\\_|___\\___|\n"
                    : ""),
            Alert.AlertType.kError);
  }

  public void setSpeeds(double intake_velocity, double extend_velocity) {
    io.intakeSpeed(intake_velocity);
    io.extendSpeed(extend_velocity);
  }

  public Command intake() {
    return Commands.run(() -> io.intakeSpeed(20), this);
  }

  public Command intakeReverse() {
    return Commands.run(() -> io.intakeSpeed(-20), this);
  }

  public Command extendHopper() {
    return Commands.run(() -> io.extendSpeed(20), this);
  }

  public Command retractHopper() {
    return Commands.run(() -> io.extendSpeed(-20), this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("rollers", inputs);
    intakeDisconnectedAlert.set(intakeConnectedDebouncer.calculate(inputs.intakeConnected));
    extendDisconnectedAlert.set(extendConnectedDebouncer.calculate(inputs.extendConnected));
  }
}
