package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter implements Subsystem {
  double abs_rel_turret_offset = 0.0;
  Rotation2d hoodAngle = Rotation2d.kZero;
  private static final double hoodLegLength1 = 4.87;
  private static final double hoodLegLength2 = 8.20;
  private static final double hoodZero = Math.asin(1.75 / 9);
  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  ShooterIO io;
  TurretIO turretIO;
  TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  Rotation2d turret_rotation = new Rotation2d();
  private static final double a_gear_ratio = 11.0 / 100.0;
  private static final double b_gear_ratio = 13.0 / 100.0;
  private static final double a_offset = 0.4;
  private static final double b_offset = 0.1;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("turret", turretInputs);
    Logger.processInputs("shooter", inputs);
    hoodAngle = fuseEncoders();
  }

  public Command fireCommand() {
    return null;
  }

  private void commandTurret(Rotation2d rotation2d) {
    rotation2d.getRotations();
  }

  // Turret, shooter
  private Pair<Rotation2d, Rotation2d> calcShootCommand(Pose2d currentPose) {
    var hubPosition = FieldConstants.hub();
    var groundDistanceToHub =
        currentPose.getTranslation().minus(hubPosition.toTranslation2d()).getNorm();
    var heightDifference = hubPosition.getZ() - 0.4;
    var exitVelocity = 5;
    var gravity = 9.81;
    var theta =
        Math.atan(
            (exitVelocity * exitVelocity
                    + Math.sqrt(
                        exitVelocity * exitVelocity * exitVelocity * exitVelocity
                            - gravity
                                * (gravity * groundDistanceToHub * groundDistanceToHub
                                    + 2 * exitVelocity * exitVelocity * heightDifference)))
                / (gravity * groundDistanceToHub));
    return new Pair<>(
        currentPose.getTranslation().minus(hubPosition.toTranslation2d()).getAngle(),
        Rotation2d.fromRadians(theta));
  }

  private Rotation2d fuseEncoders() {
    return Rotation2d.k180deg;
  }

  // both of the next functions together are just the law of cosines, bc it is a literal triangle
  private static double inverseKinematics(Rotation2d angle) {
    return Math.sqrt(
        hoodLegLength1 * hoodLegLength1
            + hoodLegLength2 * hoodLegLength2
            - 2.0 * hoodLegLength1 * hoodLegLength2 * Math.cos(angle.getRadians() - hoodZero));
  }

  private static Rotation2d forwardKinematics(double length) {
    return Rotation2d.fromRadians(
        hoodZero
            + Math.acos(
                (hoodLegLength1 * hoodLegLength1
                        + hoodLegLength2 * hoodLegLength2
                        - length * length)
                    / (2.0 * hoodLegLength1 * hoodLegLength2)));
  }

  public Shooter(ShooterIO io) {
    this.io = io;
  }
}
