// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.HoldMode;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double MIN_HOLD_VELOCITY = 0.1;
  private static final double MIN_HOLD_ROTATION = 2 * Math.PI * 0.1;
  private static final double DISTANCE_KP = 4.25;
  private static final double DISTANCE_KD = 0.15;
  private static final double ANGLE_KP = 6.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static final double LOOKAHEAD_X = 0.05;
  private static final double LOOKAHEAD_Y = 0.05;
  private static final double LOOKAHEAD_Z = 0.05;

  private static final double SHAKE_AMPLITUDE = 0.5; // m/s
  private static final double SHAKE_FREQUENCY = 3; // hz

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static Command shake(Drive drive) {
    return Commands.runEnd(() -> drive.shake = true, () -> drive.shake = false);
  }

  /**
   * Field/robot relative drive command using two joysticks (controlling linear and angular
   * velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      boolean useAsSetPosX,
      DoubleSupplier ySupplier,
      boolean useAsSetPosY,
      DoubleSupplier omegaSupplier,
      boolean useAsSetAngle,
      BooleanSupplier robotRelativeSupplier,
      BooleanSupplier snapToMajorAxis) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Rotation2d.fromDegrees(2).getRadians());

    ProfiledPIDController distanceControllerX =
        new ProfiledPIDController(
            DISTANCE_KP,
            0.0,
            DISTANCE_KD,
            new TrapezoidProfile.Constraints(
                drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearSpeedMetersPerSec()));
    ProfiledPIDController distanceControllerY =
        new ProfiledPIDController(
            DISTANCE_KP,
            0.0,
            DISTANCE_KD,
            new TrapezoidProfile.Constraints(
                drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearSpeedMetersPerSec()));
    distanceControllerX.setTolerance(0.02);
    distanceControllerY.setTolerance(0.02);

    Timer timer = new Timer();

    return Commands.run(
            () -> {
              double x = xSupplier.getAsDouble();
              double y = ySupplier.getAsDouble();
              double omega = omegaSupplier.getAsDouble();

              // Refresh values every run
              if (useAsSetAngle) drive.targetAngle = Rotation2d.fromRadians(omega);
              if (useAsSetPosX) drive.targetX = x;
              if (useAsSetPosY) drive.targetY = y;

              x = MathUtil.applyDeadband(x, DEADBAND);
              y = MathUtil.applyDeadband(y, DEADBAND);
              omega = MathUtil.applyDeadband(omega, DEADBAND);

              if (snapToMajorAxis.getAsBoolean()) {
                if (Math.abs(x) > Math.abs(y)) y = 0;
                else x = 0;
              }

              // First assume all speeds are velocity targets from joysticks, if they are not then
              // the PID controllers will directly modify the set speeds through targetSpeeds

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              if (!robotRelativeSupplier.getAsBoolean() && isFlipped) {
                x *= -1;
                y *= -1;
              }

              ChassisSpeeds targetFieldSpeeds =
                  new ChassisSpeeds(
                      Math.copySign(x * x, x) * drive.getMaxLinearSpeedMetersPerSec(),
                      Math.copySign(y * y, y) * drive.getMaxLinearSpeedMetersPerSec(),
                      Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec());

              Pose2d self = drive.getPose();

              ChassisSpeeds currentFieldSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      drive.getChassisSpeeds(), drive.getRotation());

              // ############ ROTATION AND TRANSLATION HOLDING ############
              if (!robotRelativeSupplier.getAsBoolean()) {
                if (Math.abs(omega) > DEADBAND && !useAsSetAngle) {
                  drive.holdingAngle = HoldMode.OFF;
                } else if (drive.holdingAngle == HoldMode.OFF
                    && Math.abs(currentFieldSpeeds.omegaRadiansPerSecond) < MIN_HOLD_ROTATION) {

                  drive.holdingAngle = HoldMode.HOLD;
                  drive.targetAngle =
                      useAsSetAngle
                          ? Rotation2d.fromRadians(omegaSupplier.getAsDouble())
                          : drive
                              .getRotation()
                              .plus(
                                  Rotation2d.fromRadians(
                                      currentFieldSpeeds.omegaRadiansPerSecond * LOOKAHEAD_Z));
                  angleController.reset(
                      self.getRotation().getRadians(), currentFieldSpeeds.omegaRadiansPerSecond);
                }
                if (drive.holdingAngle != HoldMode.OFF) {
                  double pidTheta =
                      angleController.calculate(
                          self.getRotation().getRadians(), drive.targetAngle.getRadians());
                  targetFieldSpeeds.omegaRadiansPerSecond = angleController.atGoal() ? 0 : pidTheta;
                }

                if ((Math.abs(x) < DEADBAND
                        && Math.abs(currentFieldSpeeds.vxMetersPerSecond) < MIN_HOLD_VELOCITY)
                    || useAsSetPosX) {
                  if (!drive.holdingX) {
                    drive.holdingX = true;
                    drive.targetX =
                        useAsSetPosX
                            ? xSupplier.getAsDouble()
                            : self.getX() + currentFieldSpeeds.vxMetersPerSecond * LOOKAHEAD_X;
                    distanceControllerX.reset(self.getX(), currentFieldSpeeds.vxMetersPerSecond);
                  }
                  double pidX = distanceControllerX.calculate(self.getX(), drive.targetX);
                  targetFieldSpeeds.vxMetersPerSecond = distanceControllerX.atGoal() ? 0 : pidX;
                } else {
                  drive.holdingX = false;
                }
                if ((Math.abs(y) < DEADBAND
                        && Math.abs(currentFieldSpeeds.vyMetersPerSecond) < MIN_HOLD_VELOCITY)
                    || useAsSetPosY) {
                  if (!drive.holdingY) {
                    drive.holdingY = true;
                    drive.targetY =
                        useAsSetPosY
                            ? ySupplier.getAsDouble()
                            : self.getY() + currentFieldSpeeds.vyMetersPerSecond * LOOKAHEAD_Y;
                    distanceControllerY.reset(self.getY(), currentFieldSpeeds.vyMetersPerSecond);
                  }
                  double pidY = distanceControllerY.calculate(self.getY(), drive.targetY);
                  targetFieldSpeeds.vyMetersPerSecond = distanceControllerY.atGoal() ? 0 : pidY;
                } else {
                  drive.holdingY = false;
                }

                targetFieldSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(targetFieldSpeeds, self.getRotation());
              } else {
                drive.holdingAngle = HoldMode.OFF;
                drive.holdingX = false;
                drive.holdingY = false;
              }

              if (drive.shake)
                targetFieldSpeeds.vyMetersPerSecond +=
                    SHAKE_AMPLITUDE * Math.sin(2.0 * Math.PI * SHAKE_FREQUENCY * timer.get());

              drive.runVelocity(targetFieldSpeeds);
            },
            drive)
        .beforeStarting(
            () -> {
              ChassisSpeeds currentSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      drive.getChassisSpeeds(), drive.getRotation());

              angleController.reset(
                  drive.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
              distanceControllerX.reset(drive.getPose().getX(), currentSpeeds.vxMetersPerSecond);
              distanceControllerY.reset(drive.getPose().getY(), currentSpeeds.vyMetersPerSecond);

              drive.resetHold();
              timer.restart();
            });
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /** Take a side-relative pose as target and drive to it */
  public static Command driveToPose(Drive drive, Pose2d target) {
    Logger.recordOutput("drive/autoAlignPose", target);
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    ProfiledPIDController distanceController =
        new ProfiledPIDController(
            DISTANCE_KP,
            0.0,
            DISTANCE_KD,
            new TrapezoidProfile.Constraints(
                drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearSpeedMetersPerSec()));
    distanceController.setTolerance(0.02);

    return Commands.run(
            () -> {
              Pose2d self = drive.getPose();

              double x = target.getX() - self.getX();
              double y = target.getY() - self.getY();

              double diff = Math.sqrt(x * x + y * y);

              double vx = 0;
              double vy = 0;

              if (distanceController.atGoal()) {
                vx = 0;
                vy = 0;
              } else if (diff > 1e-6) {
                double scalar = -distanceController.calculate(diff, 0);
                vx = (x / diff) * scalar;
                vy = (y / diff) * scalar;
              }

              double theta =
                  angleController.calculate(
                      drive.getRotation().getRadians(), target.getRotation().getRadians());
              // double theta = 0;

              ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, theta);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getPose().getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getPose().getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              distanceController.reset(
                  drive.getPose().getTranslation().getDistance(target.getTranslation()));
            });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
