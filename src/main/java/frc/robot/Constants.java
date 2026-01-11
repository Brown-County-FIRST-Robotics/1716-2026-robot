// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Constants relating to manual operation */
  public static final class Driver {
    /** The maximum lateral velocity during manual operation in m/s */
    public static final double MAX_SPEED = 6.0;
    /** The maximum angular velocity during manual operation in rad/s */
    public static final double MAX_THETA_SPEED = 9;
    /** The maximum acceleration in m/s^2 (does not include deceleration) */
    public static final double MAX_ACCELERATION = MAX_SPEED;
    /** The maximum acceleration to prevent slipping in m/s^2 */
    public static final double MAX_FRICTION_ACCELERATION = 9.8; // social studies carpet :0.5
  }

  /** Current limits for different motors */
  public static final class CurrentLimits {
    /** Current limit for NEO 550 */
    public static final int NEO550 = 20;
    /** Current limit for NEO v1.1 */
    public static final int NEO = 50;
    /** Current limit for NEO vortex */
    public static final int NEO_VORTEX = 80;
    /** Current limit for brushed motors */
    public static final int GENERIC_BRUSHED = 20;
  }

  /** The initial pose of the robot */
  public static final Pose2d INIT_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
}
