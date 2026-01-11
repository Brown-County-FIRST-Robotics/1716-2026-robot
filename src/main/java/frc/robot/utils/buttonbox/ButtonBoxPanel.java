package frc.robot.utils.buttonbox;

public abstract class ButtonBoxPanel {
  final ButtonBox bb;
  public int panel_index;

  public ButtonBoxPanel(ButtonBox bb) {
    this.bb = bb;
    bb.registerPanel(this);
  }

  abstract int getButtons();

  abstract int getAxes();

  protected boolean getButton(int index) {
    return bb.getButtonOnPanel(panel_index, index);
  }

  protected double getAxis(int index) {
    return 0.0;
  }
}
