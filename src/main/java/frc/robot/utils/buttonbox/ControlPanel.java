package frc.robot.utils.buttonbox;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlPanel extends ButtonBoxPanel {
  public ControlPanel(ButtonBox bb) {
    super(bb);
  }

  public Trigger eject(){
    return new Trigger(() -> getButton(0));
  }

  public Trigger kidModeOn(){
    return new Trigger(()-> getButton(1));
  }


  public Trigger autoAimOff(){
    return new Trigger(() -> getButton(2));
  }

  public Trigger forwardBelt(){
    return new Trigger(() -> getButton(3));
  }

  public Trigger backwardBelt(){
    return new Trigger(() -> getButton(4));
  }

  public Trigger climbUp(){//makes the robot climb up the tower
    return new Trigger(()-> getButton(5));
  }

  public Trigger climbDown(){//makes the robot climb down the tower
    return new Trigger(()-> getButton(6));
  }

  public Trigger blue(){
    return new Trigger(()->getButton(7));
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
