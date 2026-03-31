package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.OurConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  double abs_rel_turret_offset = 0.0;
  Rotation2d hoodAngle = Rotation2d.kZero;
  private static final double hoodLegLength1 = 4.87;
  private static final double hoodLegLength2 = 8.29;
  private static final double hoodZero = 0.221 + (0.221 - 0.02) + 0.02 - 0.738;
  ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  ShooterIO shooterIO;
  TurretIO turretIO;
  TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private final Debouncer shooterConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer turretConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer encoderAConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Debouncer encoderBConnectedDebouncer =
      new Debouncer(OurConstants.CONNECTED_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
  private final Alert shooterDisconnectedAlert =
      new Alert("Shooter motor disconnected", AlertType.kError);
  private final Alert turretDisconnectedAlert =
      new Alert("Turret (turn) motor disconnected", AlertType.kError);
  private final Alert encoderADisconnectedAlert =
      new Alert("Turret encoder A (11 tooth) disconnected", AlertType.kError);
  private final Alert encoderBDisconnectedAlert =
      new Alert("Turret encoder B (13 tooth) disconnected", AlertType.kError);

  public Rotation2d getTurretRotation() {
    return turret_rotation;
  }

  Rotation2d turret_rotation = new Rotation2d();
  private static final double a_gear_ratio = 11.0 / 100.0;
  private static final double b_gear_ratio = 13.0 / 100.0;
  private static final double a_offset = 0.4;
  private static final double b_offset = 0.1;

  private static final double SOFT_LIMIT_BUFFER = 0.001;

  double wdmod(double rad) {
    if (rad < 0) {
      return wdmod(rad + 2 * Math.PI);
    } else if (rad > 2 * Math.PI) {
      return wdmod(rad - 2 * Math.PI);
    }
    return rad;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("turret", turretInputs);
    Logger.processInputs("shooter", inputs);
    turret_rotation =
        fuseEncoders(turretInputs.encoder_a_position, turretInputs.encoder_b_position);
    Logger.recordOutput("turret/absoluteRotation", turret_rotation);
    turret_rotation = Rotation2d.fromRadians(wdmod(turret_rotation.getRadians()));
    Logger.recordOutput("hood", forwardKinematics(inputs.shooterHoodPosition));

    // Handle disconnection alerts
    shooterDisconnectedAlert.set(!shooterConnectedDebouncer.calculate(inputs.connected));
    turretDisconnectedAlert.set(!turretConnectedDebouncer.calculate(turretInputs.motorConnected));
    encoderADisconnectedAlert.set(
        !encoderAConnectedDebouncer.calculate(turretInputs.encoderAConnected));
    encoderBDisconnectedAlert.set(
        !encoderBConnectedDebouncer.calculate(turretInputs.encoderBConnected));
  }

  public Command fireCommand() {
    return null;
  }

  public void commandTurret(Rotation2d rotation2d) {
    double position = rotation2d.getRotations();
    if (position < 0) position += 1;
    position =
        Math.min(
            0.656148083885 - SOFT_LIMIT_BUFFER,
            Math.max(0.193516495305 + SOFT_LIMIT_BUFFER, position)); // Clamp to hardware limits
    Logger.recordOutput("turret/setAbsolutePosition", position);
    turretIO.commandPosition(
        (Rotation2d.fromRotations(position).minus(turret_rotation)).getRotations()
            + turretInputs.position);
  }

  public Translation2d getTurretTarget(Pose2d robot) {
    double distanceToCenter =
        Math.abs(8.270494 - robot.getX()); // Distance taken on the X axis (the long way)
    double distanceToMidline =
        4.034536 - robot.getY(); // Distance taken on the Y axis (the short way)
    // positive means human player side

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Translation2d targetPosition;
    if (distanceToCenter > 3.6449) targetPosition = FieldConstants.hub().toTranslation2d();
    else if (distanceToMidline > 0)
      targetPosition = new Translation2d(isFlipped ? 16.54099 - 2 : 2, 1.5);
    else targetPosition = new Translation2d(isFlipped ? 16.54099 - 2 : 2, 8.069275 - 1.5);
    Logger.recordOutput(
        "turret/autoAimPos",
        new Pose2d(targetPosition.getX(), targetPosition.getY(), Rotation2d.kZero));
    return targetPosition.minus(
        robot.plus(new Transform2d(-0.2, 0.3, Rotation2d.kZero)).getTranslation());
  }

  public void commandTurretToTrack(Pose2d p2) {
    var correctRotation = getTurretTarget(p2).getAngle().minus(p2.getRotation());
    commandTurret(correctRotation);
  }

  public void commandHoodToTrack(Pose2d p2) {
    double distanceToTarget = Units.metersToInches(getTurretTarget(p2).getNorm());
    double target = distanceToTarget * 0.0095 + 0.165;
    /* Collected data on 3/31/26
      60": 0.73
      50": 0.65
      40": 0.54
    */

    shooterIO.commandHoodPosition(target);
  }

  // Turret, shooter
  private Pair<Rotation2d, Rotation2d> calcShootCommand(Pose2d currentPose) {
    var hubPosition = FieldConstants.hub();
    var groundDistanceToHub =
        currentPose.getTranslation().minus(hubPosition.toTranslation2d()).getNorm();
    var heightDifference = hubPosition.getZ() - 0.4;
    var exitVelocity = 8;
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

  private Rotation2d fuseEncoders(double aPosition, double bPosition) {
    // A and B are read from encoder, should be somewhere from 0-1
    // A might need to be from smaller tooth gear
    double aNumOfTeeth =
        aPosition * 11; // turn encoder position to the amount of teeth the gear has rotated
    double bNumOfTeeth = bPosition * 13;
    long garbageDifference =
        (Math.round(aNumOfTeeth - bNumOfTeeth + 22) % 11); // Adding 22 to force positive after mod
    long bGearRotations =
        (garbageDifference + 11 * (garbageDifference % 2))
            / 2; // Amount of times the 13 tooth gear has made a full rotation
    double numOf100Teeth =
        bGearRotations * 13 + bNumOfTeeth; // How many teeth the 100 gear has rotated
    // Fix because the turret is currently rotated by +1 rotation
    numOf100Teeth = (numOf100Teeth + 200 + 143) % 143;
    Logger.recordOutput("turret/encoderEstNumOfTeeth", numOf100Teeth);
    double rotationInRadians = numOf100Teeth / 100 * (2 * Math.PI); // convert to radians
    return Rotation2d.fromRadians(rotationInRadians + (Math.PI) - 2.73);
  }

  // both of the next functions together are just the law of cosines, bc it is a literal triangle
  private static double inverseKinematics(Rotation2d angle) {
    return Math.sqrt(
            hoodLegLength1 * hoodLegLength1
                + hoodLegLength2 * hoodLegLength2
                - 2.0 * hoodLegLength1 * hoodLegLength2 * Math.cos(angle.getRadians() - hoodZero))
        - 4.63;
  }

  private static Rotation2d forwardKinematics(double length) {
    return Rotation2d.fromRadians(
        hoodZero
            + Math.acos(
                (hoodLegLength1 * hoodLegLength1
                        + hoodLegLength2 * hoodLegLength2
                        - (length + 4.63) * (length + 4.63))
                    / (2.0 * hoodLegLength1 * hoodLegLength2)));
  }

  public void quickWheelCommand(double volts) {
    shooterIO.quickShooter(volts);
  }

  public void setShooterSpeed(double speed) {
    shooterIO.commandShooterSpeed(speed);
  }

  public void quickServoCommand(double increment) {
    shooterIO.commandHoodPosition(increment + inputs.shooterHoodPosition);
  }

  public Shooter(ShooterIO io, TurretIO tio) {
    this.shooterIO = io;
    this.turretIO = tio;
  }
}
