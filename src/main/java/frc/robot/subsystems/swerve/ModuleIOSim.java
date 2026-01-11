package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveSimManager;

/** A simulated swerve module */
public class ModuleIOSim implements ModuleIO {
  private final int index;
  private static final Rotation2d[] chassisOffsets =
      new Rotation2d[] {
        Rotation2d.fromDegrees(-90),
        Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(90)
      };

  /**
   * Constructs a ModuleIOSim given an index (fl:0,fr:1,bl:2,br:3)
   *
   * @param index The index of the module
   */
  public ModuleIOSim(int index) {
    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.absSensorOmega = -SwerveSimManager.getInstance().getSteerVel(index);
    inputs.relativeSensorOmega = SwerveSimManager.getInstance().getSteerVel(index);
    inputs.offset = chassisOffsets[index].getRotations();
    inputs.absSensorAngle =
        SwerveSimManager.getInstance().getModPos(index).angle.unaryMinus().getRotations();
    inputs.relativeSensorAngle =
        SwerveSimManager.getInstance().getModPos(index).angle.getRotations();
    inputs.thrustPos = SwerveSimManager.getInstance().getModPos(index).distanceMeters;
    inputs.thrustVel = SwerveSimManager.getInstance().getModState(index).speedMetersPerSecond;
  }

  @Override
  public void setCmdState(double ang, double vel) {
    SwerveSimManager.getInstance()
        .commandState(index, new SwerveModuleState(vel, Rotation2d.fromRotations(ang)));
  }
}
