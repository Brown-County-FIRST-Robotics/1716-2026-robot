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
  }

  default void justSet(double volt) {}

  default void updateInputs(RollersIOInputs inputs) {}

  default void commandSpeed(double rollerVelocity, double kicker_velocity) {}
}
