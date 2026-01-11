package frc.robot.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

@Deprecated(forRemoval = true, since = "20250501")
public class Overrides {
  public static final LoggedNetworkBoolean useFieldOriented =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Use Field Oriented", true);
  public static final LoggedNetworkBoolean resetYaw =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Reset Yaw", false);
  public static final LoggedNetworkBoolean disableIMU =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable IMU", false);
  public static final LoggedNetworkBoolean disableVision =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Vision", false);
  public static final LoggedNetworkBoolean disableAutoAiming =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Auto Aiming", false);
  public static final LoggedNetworkBoolean disableAutoAlign =
      new LoggedNetworkBoolean("Shuffleboard/Overrides/Disable Auto Align", false);
}
