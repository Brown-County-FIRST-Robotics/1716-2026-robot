package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public double velocity = 0.0;
    public double temperature = 0.0;
    public double position = 0.0;
    public double current = 0.0;

    public double encoder_a_position = 0.0;
    public double encoder_b_position = 0.0;

    public boolean motorConnected = false;
    public boolean encoderAConnected = false;
    public boolean encoderBConnected = false;
  }

  // position is for motor, in rotations
  default void commandPosition(double position) {}

  default void updateInputs(TurretIOInputs inputs) {}
}
