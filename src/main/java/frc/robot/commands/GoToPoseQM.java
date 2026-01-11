package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GoToPoseQM extends Command {
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.7, 0.01, 0.0, new TrapezoidProfile.Constraints(1.0, 3.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.7, 0.01, 0.0, new TrapezoidProfile.Constraints(1.0, 3.0), 0.02);
  final SwerveDrivetrain drivetrain;
  final Supplier<Pose2d> targeter;
  Pose2d target;

  public GoToPoseQM(SwerveDrivetrain drivetrain, Supplier<Pose2d> targeter) {
    this.drivetrain = drivetrain;
    this.targeter = targeter;
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static final double drivekP = 0.8;
  private static final double thetakP = 4.0;
  private static final double driveMaxVelocity = 3.8;
  private static final double driveMaxVelocitySlow = 1;
  private static final double driveMaxAcceleration = 3.0;
  private static final double thetaMaxVelocity = 6.28;
  private static final double thetaMaxAcceleration = 8.0;
  private static final double driveTolerance = 0.001;
  private static final double thetaTolerance = 6.28 / 360.0;
  private static final double ffMinRadius = 0.05;
  private static final double ffMaxRadius = 0.1;

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;

  private final Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private final DoubleSupplier omegaFF = () -> 0.0;

  @Override
  public void initialize() {
    target = targeter.get();
    Pose2d currentPose = drivetrain.getPosition();
    ChassisSpeeds fieldVelocity = drivetrain.getVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    driveController.setP(drivekP);
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    driveController.setTolerance(driveTolerance);
    thetaController.setP(thetakP);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    thetaController.setTolerance(thetaTolerance);
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers

    // Get current pose and target pose
    Pose2d currentPose = drivetrain.getPosition();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(target.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(target.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                target.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - target.getTranslation().getY(),
                        currentPose.getTranslation().getX() - target.getTranslation().getX())))
            .transformBy(toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), target.getRotation().getRadians());
    thetaErrorAbs = Math.abs(currentPose.getRotation().minus(target.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - target.getTranslation().getY(),
                        currentPose.getTranslation().getX() - target.getTranslation().getX())))
            .transformBy(toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity = driveVelocity.interpolate(Translation2d.kZero.times(5.0), linearS);
    thetaVelocity = MathUtil.interpolate(thetaVelocity, 0.0 * 6.0, thetaS);

    // Command speeds
    drivetrain.humanDrive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {target});
  }

  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, Rotation2d.kZero);
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  @Override
  public boolean isFinished() {
    Twist2d err = drivetrain.getPosition().log(target);
    return false; // Math.abs(err.dtheta) < 0.1 && (err.dx * err.dx + err.dy * err.dy) < 0.1;
  }
}
