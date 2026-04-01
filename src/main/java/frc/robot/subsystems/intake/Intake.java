package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
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
  double extendedZeroPosition;

  public Intake(IntakeIO io) {
    this.io = io;
    intakeDisconnectedAlert = new Alert("Intake motor disconnected", Alert.AlertType.kError);
    extendDisconnectedAlert =
        new Alert("Intake extension motor disconnected", Alert.AlertType.kError);
  }

  public double getRealExtenderPosition() {
    return inputs.extendPosition - extendedZeroPosition;
  }

  public void setZeroPosition() {
    extendedZeroPosition = inputs.extendPosition;
  }

  public void setSpeeds(double intake_velocity, double extend_velocity) {
    io.intakeSpeed(intake_velocity);
    io.extendSpeed(extend_velocity);
  }

  public Command intake() {
    return Commands.run(() -> io.intakeSpeed(60), this);
  }

  public Command intakeReverse() {
    return Commands.run(() -> io.intakeSpeed(-40), this);
  }

  public Command intakeStop() {
    return Commands.run(() -> io.intakeSpeed(0), this);
  }

  public Command shake() {
    Timer time = new Timer();
    boolean[] isMovingIn = {true};
    return Commands.run(
            () -> {
              if (isMovingIn[0]) {
                io.extenderVelocity(-1);
                if (time.hasElapsed(0.4)) {
                  isMovingIn[0] = false;
                  time.restart();
                }
              } else {
                io.extenderVelocity(1);
                if (time.hasElapsed(0.3)) {
                  isMovingIn[0] = true;
                  time.restart();
                }
              }
            },
            this)
        .beforeStarting(
            () -> {
              isMovingIn[0] = true;
              time.restart();
            });
  }

  public Command extendHopper() {
    return Commands.runOnce(() -> io.extenderPosition(extendedZeroPosition + 12.75), this);
  }

  public Command retractHopper() {
    return Commands.runOnce(() -> io.extenderPosition(extendedZeroPosition), this);
  }

  public Command extendHopperVelocity(double speed) {
    return Commands.runEnd(() -> io.extenderVelocity(speed), () -> io.extenderVelocity(0), this);
  }

  public Command retractHopperVelocity(double speed) {
    return Commands.runEnd(() -> io.extenderVelocity(-speed), () -> io.extenderVelocity(0), this);
  }

  public Command retractStop() {
    return Commands.run(() -> io.extenderVelocity(0), this);
  }

  public boolean isExtenderConnected() {
    return inputs.extendConnected;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake", inputs);
    Logger.recordOutput("intake/distanceToZero", getRealExtenderPosition());
    intakeDisconnectedAlert.set(!intakeConnectedDebouncer.calculate(inputs.intakeConnected));
    extendDisconnectedAlert.set(!extendConnectedDebouncer.calculate(inputs.extendConnected));
  }
}
