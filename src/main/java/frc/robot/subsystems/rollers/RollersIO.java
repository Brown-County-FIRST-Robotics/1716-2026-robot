package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public double rollerVelocity = 0.0;
    public boolean rollersConnected = false;
    public double appliedVolts = 0.0;
    public double appliedCurrent = 0.0;
  }

  default void updateInputs(RollersIOInputs inputs) {}

  default void commandSpeed(double velocity) {}
}
