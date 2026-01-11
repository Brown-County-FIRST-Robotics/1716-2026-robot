package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class Vector {
  private double magnitude;
  private Rotation2d angle;

  /** Makes a zero vector */
  private Vector() {
    magnitude = 0;
    angle = Rotation2d.fromDegrees(0);
  }

  private Vector(double magnitude, Rotation2d angle) {
    this.magnitude = magnitude;
    this.angle = angle;
  }

  private Vector(double x, double y) {
    magnitude = Math.hypot(x, y);
    if (x == 0 && y == 0) {
      x = 0.00000001;
    }
    if (magnitude == 0) {
      angle = new Rotation2d(0);
    } else {
      angle = new Rotation2d(x, y);
    }
  }

  public static Vector zeroVector() {
    return new Vector();
  }

  public static Vector fromPolar(double magnitude, Rotation2d angle) {
    return new Vector(magnitude, angle);
  }

  public static Vector fromCartesian(double x, double y) {
    return new Vector(x, y);
  }

  public double getNorm() {
    return magnitude;
  }

  public void setNorm(double norm) {
    magnitude = norm;
  }

  public Rotation2d getAngle() {
    return angle;
  }

  public void setAngle(Rotation2d rotation2d) {
    angle = rotation2d;
  }

  public double getX() {
    return angle.getCos() * magnitude;
  }

  public void setX(double x) {
    setRectangular(x, getY());
  }

  public double getY() {
    return angle.getSin() * magnitude;
  }

  public void setY(double y) {
    setRectangular(getX(), y);
  }

  public void setRectangular(double x, double y) {
    magnitude = Math.hypot(x, y);
    angle = new Rotation2d(x, y);
  }

  public Vector plus(Vector toAdd) {
    return fromCartesian(getX() + toAdd.getX(), getY() + toAdd.getY());
  }

  public Vector minus(Vector toSubtract) {
    return fromCartesian(getX() - toSubtract.getX(), getY() - toSubtract.getY());
  }

  public Vector scale(double scaleFactor) {
    return fromPolar(magnitude * scaleFactor, angle);
  }

  public void scaleSelf(double scaleFactor) {
    magnitude *= scaleFactor;
  }
}
