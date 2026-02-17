package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;
    public double shooterVelocity = 0;
    public double shooterAppliedOutput = 0;
    public double shooterCurrent = 0;
    public double shooterHoodPosition = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void commandShooterSpeed(double speed) {}

  public default void commandHoodPosition(double position) {}
}
