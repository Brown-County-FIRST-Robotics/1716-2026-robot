package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocity = 0.0;
    public boolean intakeConnected = false;
    public double intakeAppliedVolts = 0.0;
    public double intakeAppliedCurrent = 0.0;
    public double extendVelocity = 0.0;
    public boolean extendConnected = false;
    public double extendAppliedVolts = 0.0;
    public double extendAppliedCurrent = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void intakeSpeed(double intake_speed) {}

  default void extendSpeed(double extend_speed) {}
}
