package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlPanel extends ButtonBoxPanel {
  public ControlPanel(ButtonBox bb) {
    super(bb);
  }

  public Trigger questDown() {
    return new Trigger(() -> getButton(4));
  }

  public Trigger kidModeOn() {
    return new Trigger(() -> getButton(0));
  }

  public Trigger hopperOut() { // extend the hopper
    return new Trigger(() -> getButton(6));
  }

  public Trigger hopperIn() { // retract the hopper
    return new Trigger(() -> getButton(5));
  }

  public Trigger intakeForward() { // intake into the robot
    return new Trigger(() -> getButton(7));
  }

  public Trigger intakeReverse() { // spit balls out
    return new Trigger(() -> getButton(8));
  }

  // public Trigger intakeStop() {
  //   return new Trigger(() -> getButton(6));
  // }

  public Trigger autoAlignTrenchLeft() {
    return new Trigger(() -> getButton(2));
  }

  public Trigger autoAlignTrenchRight() {
    return new Trigger(() -> getButton(3));
  }

  // public Trigger turretAutoAimOn() {
  //   return new Trigger(() -> getButton(9));
  // }

  public Trigger hoodSafePosition() {
    return new Trigger(() -> getButton(10));
  }

  public Trigger hoodShootPosition() {
    return new Trigger(() -> getButton(11));
  }

  @Override
  int getButtons() {
    return 12;
  }

  @Override
  int getAxes() {
    return 0;
  }
}
