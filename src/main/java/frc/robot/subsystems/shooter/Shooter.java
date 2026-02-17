package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter implements Subsystem {
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("shooter", inputs);
  }

  Rotation2d hoodAngle = Rotation2d.kZero;
  private static double hoodLegLength1 = 4.87;
  private static double hoodLegLength2 = 8.20;
  // both of the next functions together are just the law of cosines, bc it is a literal triangle
  private static double inverseKinematics(double angle) {

    return Math.sqrt(
        hoodLegLength1 * hoodLegLength1
            + hoodLegLength2 * hoodLegLength2
            - 2.0 * hoodLegLength1 * hoodLegLength2 * Math.cos(angle));
  }

  private static double forwardKinematics(double length) {
    return Math.acos(
        (hoodLegLength1 * hoodLegLength1 + hoodLegLength2 * hoodLegLength1 - length * length)
            / (2.0 * hoodLegLength1 * hoodLegLength2));
  }

  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }
}
