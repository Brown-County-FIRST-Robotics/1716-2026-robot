package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/** Field positions of game components */
public class FieldConstants {
  public static final AprilTagFieldLayout layout=AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public static final double fieldLength = layout.getFieldLength(); // meters
  public static final double fieldWidth = layout.getFieldWidth();

  /**
   * Flips the translation based on alliance
   *
   * @param inp The position for the blue alliance
   * @return The position for the FMS alliance
   */
  public static Translation3d flip(Translation3d inp) {
    return new Translation3d(
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
            ? inp.getX()
            : fieldLength - inp.getX(),
        inp.getY(),
        inp.getZ());
  }

  /**
   * Flips the rotation based on alliance
   *
   * @param inp The rotation when on the blue alliance
   * @return The rotation for the FMS alliance
   */
  public static Rotation2d flip(Rotation2d inp) {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue
        ? inp
        : new Rotation2d(-inp.getCos(), inp.getSin());
  }
  /**
   * Flips the pose based on alliance
   *
   * @param inp The pose when on the blue alliance
   * @return The pose for the FMS alliance
   */
  public static Pose2d flip(Pose2d inp) {
    return new Pose2d(flip(inp.getTranslation()), flip(inp.getRotation()));
  }

  /**
   * Flips the translation based on alliance
   *
   * @param inp The position for the blue alliance
   * @return The position for the FMS alliance
   */
  public static Translation2d flip(Translation2d inp) {
    return flip(new Translation3d(inp.getX(), inp.getY(), 0)).toTranslation2d();
  }
}
