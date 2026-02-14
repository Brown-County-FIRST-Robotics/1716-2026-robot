package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  ShooterIO shooterIO;
  TurretIO turretIO;
  TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  Rotation2d turret_rotation = new Rotation2d();
  static final double error_tolerance = 0.01;
  final CustomAlerts.TimeLatchAlert encoderFuckupAlert =
      new CustomAlerts.TimeLatchAlert(Alert.AlertType.kWarning, 2, "Turret position not solvable");
  static final double a_gear_ratio = 11.0 / 100.0;
  static final double b_gear_ratio = 13.0 / 100.0;
  static final double a_offset = 0.4;
  static final double b_offset = 0.1;
  static final double shooterLeg1Length = 4.87;
  static final double shooterLeg2Length = 8.20;

  /**
   * Well-defined modulo-one function for doubles. If the value is >=1, it decrements it. If the
   * value is <0, it increments it.
   */
  double wd_mod(double x) {
    if (x >= 1) {
      return wd_mod(x - 1);
    } else if (x < 0) {
      return wd_mod(x + 1);
    } else {
      return x;
    }
  }

  /** Returns the distance between the two values on a circle. */
  double circular_diff(double x, double y) {
    x = wd_mod(x);
    y = wd_mod(y);
    return Math.min(Math.min(Math.abs(1 + x - y), Math.abs(x - y)), Math.abs(x - y - 1));
  }

  /** The average of the two values on a circle. */
  double circular_avg(double x, double y) {
    x = wd_mod(x);
    y = wd_mod(y);
    if (Math.abs(x - y) > 0.5) {
      y += 1;
    }
    if (Math.abs(x - y) > 0.5) {
      y -= 1;
      x += 1;
    }
    return wd_mod((x + y) / 2.0);
  }

  /**
   * Fuses two encoders that are geared to a central gear with relatively prime ratios
   *
   * @param a Position of encoder a
   * @param b Position of encoder b
   * @return The fused position and the error.
   */
  Pair<Double, Double> fuseEncoders(double a, double b) {
    var a_turret = wd_mod(a - a_offset) * a_gear_ratio;
    var b_turret = wd_mod(b - b_offset) * b_gear_ratio;
    double[] possible_a_rots =
        new double[] {
          wd_mod(a_turret + a_gear_ratio * 0),
          wd_mod(a_turret + a_gear_ratio * 1),
          wd_mod(a_turret + a_gear_ratio * 2),
          wd_mod(a_turret + a_gear_ratio * 3),
          wd_mod(a_turret + a_gear_ratio * 4),
          wd_mod(a_turret + a_gear_ratio * 5),
          wd_mod(a_turret + a_gear_ratio * 6),
          wd_mod(a_turret + a_gear_ratio * 7),
          wd_mod(a_turret + a_gear_ratio * 8),
          wd_mod(a_turret + a_gear_ratio * 9),
          wd_mod(a_turret + a_gear_ratio * 10),
          wd_mod(a_turret + a_gear_ratio * 11),
          wd_mod(a_turret + a_gear_ratio * 12),
          wd_mod(a_turret + a_gear_ratio * 13),
          wd_mod(a_turret + a_gear_ratio * 14),
          wd_mod(a_turret + a_gear_ratio * 15),
          wd_mod(a_turret + a_gear_ratio * 16),
        };
    double[] possible_b_rots =
        new double[] {
          wd_mod(b_turret + b_gear_ratio * 0),
          wd_mod(b_turret + b_gear_ratio * 1),
          wd_mod(b_turret + b_gear_ratio * 2),
          wd_mod(b_turret + b_gear_ratio * 3),
          wd_mod(b_turret + b_gear_ratio * 4),
          wd_mod(b_turret + b_gear_ratio * 5),
          wd_mod(b_turret + b_gear_ratio * 6),
          wd_mod(b_turret + b_gear_ratio * 7),
          wd_mod(b_turret + b_gear_ratio * 8),
          wd_mod(b_turret + b_gear_ratio * 9),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 11),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 10),
          wd_mod(b_turret + b_gear_ratio * 12)
        };
    var best_error = Double.MAX_VALUE;
    var best_val = 0.0;
    for (double possibleARot : possible_a_rots) {
      for (double possibleBRot : possible_b_rots) {
        var err = circular_diff(possibleARot, possibleBRot);
        var val = circular_avg(possibleARot, possibleBRot);
        if (err < best_error) {
          best_error = err;
          best_val = val;
        }
      }
    }
    return Pair.of(best_val, best_error);
  }

  double kinematics(double length) {
    return Math.acos(
        (shooterLeg1Length * shooterLeg1Length
                + shooterLeg2Length * shooterLeg2Length
                - length * length)
            / (2.0 * shooterLeg1Length * shooterLeg2Length));
  }

  double inverse_kinematics(double angle) {
    return Math.sqrt(
        shooterLeg1Length * shooterLeg1Length
            + shooterLeg2Length * shooterLeg2Length
            - 2.0 * shooterLeg1Length * shooterLeg2Length * Math.cos(angle));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Turret", turretInputs);
    var fusion = fuseEncoders(turretInputs.encoder_a_position, turretInputs.encoder_b_position);
    Logger.recordOutput("Shooter/FusedEncoderError", fusion.getSecond());
    if (fusion.getSecond() > error_tolerance) {
      encoderFuckupAlert.latch();
    } else {
      turret_rotation = Rotation2d.fromRotations(fusion.getFirst());
    }
    Logger.recordOutput("Shooter/FusedTurretRotation", turret_rotation);
  }
}
