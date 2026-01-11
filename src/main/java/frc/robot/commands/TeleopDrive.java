package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.utils.DualRateLimiter;
import frc.robot.utils.Overrides;
import frc.robot.utils.Vector;
import frc.robot.utils.buttonbox.OverridePanel;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** A command for manual control */
public class TeleopDrive extends Command {
  private final SwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private final Optional<CommandXboxController> secondController;
  private final OverridePanel overridePanel;

  boolean doFieldOriented = true;
  boolean locked = false; // point wheels towards center in x pattern
  final DualRateLimiter translationLimiter =
      new DualRateLimiter(6, 100); // translational velocity rate limiter (ie:acceleration limiter)
  final DualRateLimiter rotationLimiter =
      new DualRateLimiter(
          8, 100); // angular velocity limiter (omega) (ie:angular acceleration limiter)

  private static final double deadbandSize = 0.08;
  public boolean isKidMode = false;

  public double getKidModeSpeed() {
    return kidModeSpeed;
  }

  public void setKidModeSpeed(double newKidModeSpeed) {
    if (newKidModeSpeed > 0 && newKidModeSpeed <= Constants.Driver.MAX_SPEED) {
      this.kidModeSpeed = newKidModeSpeed;
    }
  }

  private double kidModeSpeed = 1.0;

  // These are class fields instead of local variables for Garbage collector reasons, although I
  // doubt it is a big deal

  double slowModeSpeedModifier = 0.0;
  ChassisSpeeds commandedSpeeds = new ChassisSpeeds(0, 0, 0);
  ChassisSpeeds finalSpeeds = new ChassisSpeeds(0, 0, 0);
  Vector previousCommand = Vector.zeroVector();

  public TeleopDrive(
      SwerveDrivetrain drivetrain, CommandXboxController controller, OverridePanel overridePanel_) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.secondController = Optional.empty();
    this.overridePanel = overridePanel_;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    translationLimiter.reset(0);
    rotationLimiter.reset(0);
  }

  @Override
  public void execute() {
    isKidMode = overridePanel.kidMode().getAsBoolean();
    Logger.recordOutput("iskidmode", isKidMode);
    Logger.recordOutput("kidmodespeed", kidModeSpeed);
    slowModeSpeedModifier = controller.getHID().getLeftBumperButton() ? 0.2 : 1.0;
    if (isKidMode
        && secondController.isPresent()
        && secondController.get().rightTrigger().getAsBoolean()) {
      slowModeSpeedModifier = 0;
    }
    doFieldOriented = !controller.getHID().getRightBumperButton() && !isKidMode;
    locked = false;
    commandedSpeeds =
        new ChassisSpeeds(
            deadScale(controller.getLeftY()),
            deadScale(controller.getLeftX()),
            rotationLimiter.calculate(
                deadScale(controller.getRightX())
                    * Constants.Driver.MAX_THETA_SPEED
                    * slowModeSpeedModifier
                    * (isKidMode
                        ? 0.2
                        : 1))); // This needs to be a different type, the speeds need to be
    // percentage at this step, not velocity
    // TODO: figure out what I meant by that

    if (doFieldOriented) {
      Rotation2d currentRotation =
          DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
                  == DriverStation.Alliance.Red
              ? drivetrain.getPosition().getRotation()
              : drivetrain
                  .getPosition()
                  .getRotation()
                  .rotateBy(
                      Rotation2d.fromRotations(0.5)); // current rotation relative to the driver
      commandedSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              new ChassisSpeeds(
                  commandedSpeeds.vxMetersPerSecond,
                  commandedSpeeds.vyMetersPerSecond,
                  -commandedSpeeds.omegaRadiansPerSecond),
              currentRotation);
    } else {
      commandedSpeeds =
          new ChassisSpeeds(
              -commandedSpeeds.vxMetersPerSecond,
              -commandedSpeeds.vyMetersPerSecond,
              -commandedSpeeds.omegaRadiansPerSecond);
    }

    Vector commandedVector =
        Vector.fromCartesian(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);

    double angle = commandedVector.getAngle().getDegrees();
    int angleLockDegrees = 5;
    if (angle < -180 + angleLockDegrees || angle > 180 - angleLockDegrees) {
      commandedVector.setAngle(Rotation2d.fromDegrees(180));
    } else if (angle > -90 - angleLockDegrees && angle < -90 + angleLockDegrees) {
      commandedVector.setAngle(Rotation2d.fromDegrees(-90));
    } else if (angle > -angleLockDegrees && angle < angleLockDegrees) {
      commandedVector.setAngle(Rotation2d.fromDegrees(0));
    } else if (angle > 90 - angleLockDegrees && angle < 90 + angleLockDegrees) {
      commandedVector.setAngle(Rotation2d.fromDegrees(90));
    }

    commandedVector.setNorm(clamp(commandedVector.getNorm(), 1.0));
    commandedVector.setNorm(
        commandedVector.getNorm() * Math.abs(commandedVector.getNorm())); // square it
    commandedVector.setNorm(
        commandedVector.getNorm()
            * (isKidMode ? kidModeSpeed : Constants.Driver.MAX_SPEED)
            * slowModeSpeedModifier); // convert to m/s from percent

    // make sure command never gets too far from reality
    Vector realVelocity =
        Vector.fromCartesian(
            drivetrain.getVelocity().vxMetersPerSecond, drivetrain.getVelocity().vyMetersPerSecond);
    Vector currentRealityDistortion = previousCommand.minus(realVelocity);
    Vector currentVector =
        realVelocity.plus(
            Vector.fromPolar(
                clamp(currentRealityDistortion.getNorm(), Constants.Driver.MAX_SPEED / 5.0),
                currentRealityDistortion.getAngle()));

    Vector velocityChange = commandedVector.minus(currentVector);
    double frictionClampedVelocityChange =
        clamp(
            velocityChange.getNorm(),
            Constants.Driver.MAX_FRICTION_ACCELERATION / 50); // TODO: CHANGE NAME
    Vector cappedAcceleration =
        Vector.fromPolar(frictionClampedVelocityChange, velocityChange.getAngle());
    commandedVector = currentVector.plus(cappedAcceleration);

    double jeff = commandedVector.getNorm() - currentVector.getNorm();
    if (jeff > Constants.Driver.MAX_ACCELERATION / 50) {
      commandedVector.setNorm(currentVector.getNorm() + Constants.Driver.MAX_ACCELERATION / 50);
    }

    Logger.recordOutput("Current Speed", currentVector.getNorm());

    drivetrain.humanDrive(
        ChassisSpeeds.discretize(
            new ChassisSpeeds(
                commandedVector.getX(),
                commandedVector.getY(),
                isKidMode
                        && secondController.isPresent()
                        && secondController.get().rightTrigger().getAsBoolean()
                    ? 0
                    : commandedSpeeds.omegaRadiansPerSecond),
            0.02));
    previousCommand = commandedVector;

    if (controller.getHID().getBackButtonPressed()) {
      drivetrain.setPosition(
          new Pose2d(drivetrain.getPosition().getTranslation(), Rotation2d.fromRotations(0.5)));
    }

    doFieldOriented = Overrides.useFieldOriented.get();
    locked = controller.getHID().getXButtonPressed() || locked;
    if (locked) {
      drivetrain.lockWheels();
    }

    Logger.recordOutput("TeleopDrive/locked", locked);
    Logger.recordOutput("TeleopDrive/foc", doFieldOriented);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.humanDrive(new ChassisSpeeds());
  }

  /**
   * Checks if the value is in the deadband
   *
   * @param val The value to check
   * @return Whether val is in the deadband
   */
  static boolean withinDeadband(double val) {
    return Math.abs(val) < deadbandSize;
  }

  /**
   * Applies a deadband, then scales the resultant value to make output continuous
   *
   * @param val The value to scale
   * @return The value with the deadband applied
   */
  public static double deadScale(double val) {
    return withinDeadband(val)
        ? 0
        : (val > 0
            ? (val - deadbandSize) / (1 - deadbandSize)
            : (val + deadbandSize) / (1 - deadbandSize));
  }

  // Clamps the value to the max, applies in both negative and positive
  private double clamp(double x, double max) {
    if (x > max) {
      x = max;
    } else if (x < -max) {
      x = -max;
    }
    return x;
  }
}
