package frc.robot;

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

  public static Pose2d getFace(int faceID) {

    return flip(
        switch (faceID) {
          case 0 -> new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
          case 1 -> new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
          case 2 -> new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
          case 3 -> new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
          case 4 -> new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
          case 5 -> new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));
          default -> Pose2d.kZero;
        });
  }

  public static Rectangle2d getBox(int boxID) {
    double rectLength = 2.0;
    return new Rectangle2d(
        getFace(boxID).transformBy(new Transform2d(rectLength / 2, 0.0, Rotation2d.kZero)),
        rectLength,
        Units.inchesToMeters(36.792600));
  }

  public static Pose2d getPole(int index, boolean isLeft) {
    return getFace(index)
        .transformBy(
            new Transform2d(0.0254 * 14.0, 0.1559 * (isLeft ? -1.0 : 1.0), Rotation2d.k180deg));
  }

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
            : 17.548 - inp.getX(),
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
