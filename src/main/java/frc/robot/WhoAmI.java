package frc.robot;

import java.util.Objects;

/** The class that contains the information about whom the robot is */
public final class WhoAmI {
  /** The mode of the robot */
  public static final Mode mode = Mode.REAL;
  /** The robot */
  public static final RobotType bot = RobotType.SWERVEBASE;
  /** The appendages to the robot */
  public static final Appendages[] appendages = {
    Appendages.ELEVATOR, Appendages.WRIST, Appendages.GRIPPER
  };

  public static final boolean isDemoMode = false;

  /** The robot types */
  public enum RobotType {
    /** he mecanum robot */
    @Deprecated(forRemoval = true, since = "20250501")
    MECHBASE,
    /** A simulated swerve robot */
    SIMSWERVEBASE,
    /** The swerve robot */
    SWERVEBASE
  }

  /** The appendages to the robot */
  public enum Appendages {
    ELEVATOR,
    WRIST,
    GRIPPER,
  }

  /** The code execution mode */
  public enum Mode {
    /** A real robot */
    REAL,
    /** Log file replay */
    REPLAY,
    /** Simulated */
    SIM
  }

  private static void checkSim() {
    if (mode == Mode.REAL) {
      throw new IllegalArgumentException("Cannot deploy code in Sim mode to the robot");
    }
    if (mode == Mode.SIM) {
      if (bot != RobotType.SIMSWERVEBASE) {
        throw new IllegalArgumentException(
            "You are currently deploying code meant for a real robot to a simulator");
      }
      for (var appendage : appendages) {
        switch (appendage) {
          case GRIPPER, ELEVATOR, WRIST -> throw new IllegalArgumentException(
              "You are currently deploying code meant for a real robot to a simulator");
        }
      }
    }
  }

  private static void checkReal() {
    if (mode != Mode.REAL) {
      throw new IllegalArgumentException("Cannot deploy code in Sim mode to the robot");
    }
    boolean override = false; // Make true to override deploy checking
    if (bot == RobotType.SIMSWERVEBASE && !override) {
      throw new IllegalArgumentException(
          "You are currently deploying code meant for the simulator to a real robot. ONLY DO THIS IF YOU ABSOLUTELY KNOW WHAT YOU ARE DOING. ");
    }
  }

  /**
   * Checks the configuration
   *
   * @param args Not used
   */
  public static void main(String... args) {
    if (args.length != 1) {
      throw new IllegalArgumentException("Give me arguments");
    }
    if (Objects.equals(args[0], "sim")) {
      checkSim();
    } else if (Objects.equals(args[0], "real")) {
      checkReal();
    } else {
      throw new IllegalArgumentException("Invalid arguments");
    }
  }
}
