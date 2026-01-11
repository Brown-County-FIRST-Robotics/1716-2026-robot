package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManipulatorPanel extends ButtonBoxPanel {
  public ManipulatorPanel(ButtonBox bb) {
    super(bb);
  }

  public Trigger trough() { // level 1 coral
    return new Trigger(() -> getButton(0));
  }

  public Trigger level2() {
    return new Trigger(() -> getButton(1));
  }

  public Trigger level3() {
    return new Trigger(() -> getButton(2));
  }

  public Trigger level4() {
    return new Trigger(() -> getButton(3));
  }

  public Trigger algaeLow() {
    return new Trigger(() -> getButton(4));
  }

  public Trigger algaeHigh() {
    return new Trigger(() -> getButton(5));
  }

  public Trigger leftPole() {
    return new Trigger(() -> getButton(6));
  }

  public Trigger rightPole() {
    return new Trigger(() -> getButton(6 + 1));
  }

  // checks if both or neither pole buttons are pressed
  public Trigger noPole() {
    return new Trigger(() -> (!getButton(6) && !getButton(7)) || (getButton(6) && getButton(7)));
  }

  public Trigger processor() {
    return new Trigger(() -> getButton(3));
  }

  public Trigger intake() { // coral station
    return new Trigger(() -> getButton(8));
  }

  public Trigger eject() {
    return new Trigger(() -> getButton(9));
  }

  @Override
  int getButtons() {
    return 10;
  }

  @Override
  int getAxes() {
    return 0;
  }
}
