package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OverridePanel extends ButtonBoxPanel {
  public OverridePanel(ButtonBox bb) {
    super(bb);
  }

  public Trigger disableFOC() {
    return new Trigger(() -> getButton(1));
  }

  public Trigger kidMode() {
    // TODO: this will be on another panel in the future
    return new Trigger(() -> getButton(4));
  }

  public Trigger resetPosToSpeaker() {
    return new Trigger(() -> getButton(6));
  }

  public Trigger justFire() {
    return new Trigger(() -> getButton(7));
  }

  @Override
  int getButtons() {
    return 8;
  }

  @Override
  int getAxes() {
    return 0;
  }
}
