package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualMotorPanel extends ButtonBoxPanel {
  public ManualMotorPanel(ButtonBox bb) {
    super(bb);
  }

  Trigger left(int ind) {
    return new Trigger(() -> getButton(ind * 2));
  }

  Trigger right(int ind) {
    return new Trigger(() -> getButton(ind * 2 + 1));
  }

  Trigger either(int ind) {
    return left(ind).or(right(ind));
  }

  @Override
  int getButtons() {
    return 16;
  }

  @Override
  int getAxes() {
    return 0;
  }
}
