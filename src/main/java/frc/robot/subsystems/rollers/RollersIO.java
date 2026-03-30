package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public double rollerVelocity = 0.0;
    public boolean rollersConnected = false;
    public double rollerAppliedVolts = 0.0;
    public double rollerAppliedCurrent = 0.0;

    public double kickerVelocity = 0.0;
    public boolean kickerConnected = false;
    public double kickerAppliedVolts = 0.0;
    public double kickerAppliedCurrent = 0.0;

    public double kicklerVelocity = 0.0;
    public boolean kicklerConnected = false;
    public double kicklerAppliedVolts = 0.0;
    public double kicklerAppliedCurrent = 0.0;
  }

  default void updateInputs(RollersIOInputs inputs) {}

  default void commandSpeed(
      double kickler_velocity, double roller_velocity, double kicker_velocity) {}
}
