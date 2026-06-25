package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public double starsVelocity = 0.0;
    public boolean starsConnected = false;
    public double starsAppliedVolts = 0.0;
    public double starsAppliedCurrent = 0.0;

    public double kickerVelocity = 0.0;
    public boolean kickerConnected = false;
    public double kickerAppliedVolts = 0.0;
    public double kickerAppliedCurrent = 0.0;

    public double beltsVelocity = 0.0;
    public boolean beltsConnected = false;
    public double beltsAppliedVolts = 0.0;
    public double beltsAppliedCurrent = 0.0;
  }

  default void updateInputs(RollersIOInputs inputs) {}

  default void commandSpeed(double belts_velocity, double stars_velocity, double kicker_velocity) {}
}
