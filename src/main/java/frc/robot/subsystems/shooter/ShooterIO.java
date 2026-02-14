package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooter_velocity = 0.0;
    public double shooter_temperature = 0.0;
    public double shooter_current = 0.0;
    public double aimer_position = 0.0;

    public boolean motorConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}
}
