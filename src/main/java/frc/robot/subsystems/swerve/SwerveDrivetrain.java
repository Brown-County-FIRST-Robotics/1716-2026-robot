package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.utils.Overrides;
import frc.robot.utils.PoseEstimator;
import org.littletonrobotics.junction.Logger;

/** The swerve drivetrain subsystem */
public class SwerveDrivetrain implements Subsystem {
  private static final double WHEEL_SPACING = 21 * 0.0254;
  private static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_SPACING / 2, WHEEL_SPACING / 2),
          new Translation2d(WHEEL_SPACING / 2, -WHEEL_SPACING / 2),
          new Translation2d(-WHEEL_SPACING / 2, WHEEL_SPACING / 2),
          new Translation2d(-WHEEL_SPACING / 2, -WHEEL_SPACING / 2));
  private static final double MAX_WHEEL_SPEED = 5.0;
  final Module fl;
  final Module fr;
  final Module bl;
  final Module br;

  Rotation3d lastIMU;
  SwerveModulePosition[] lastPositions;
  final PoseEstimator poseEstimator;

  final IMUIO imu;
  final IMUIOInputsAutoLogged imuInputs = new IMUIOInputsAutoLogged();

  private SwerveModulePosition[] getPositions() {

    return new SwerveModulePosition[] {
      fl.getChassisRelativePosition(),
      fr.getChassisRelativePosition(),
      bl.getChassisRelativePosition(),
      br.getChassisRelativePosition()
    };
  }

  /**
   * Creates a SwerveDrivetrain from IO
   *
   * @param fl Front left module IO
   * @param fr Front right module IO
   * @param bl Back left module IO
   * @param br Back right module IO
   * @param imu IMU IO
   */
  public SwerveDrivetrain(Module fl, Module fr, Module bl, Module br, IMUIO imu) {
    this.imu = imu;
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
    poseEstimator = new PoseEstimator();
    //    poseEstimator.setPose(Constants.INIT_POSE);
    lastIMU = getGyro();
    lastPositions = getPositions();
  }

  public void periodic() {
    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
    fl.periodic();
    fr.periodic();
    bl.periodic();
    br.periodic();

    Logger.recordOutput("Drive/RealStates", getWheelSpeeds());
    Twist2d odoTwist = KINEMATICS.toTwist2d(lastPositions, getPositions());
    if (!Overrides.disableIMU.get()) {
      var omega = -getGyro().minus(lastIMU).getX();
      poseEstimator.setDelta(omega);
    }
    lastPositions = getPositions();
    lastIMU = getGyro();
    Logger.recordOutput("Drive/Pose", getPosition());

    checkForYawReset();
  }

  private void checkForYawReset() {
    if (Overrides.resetYaw.get()) {
      poseEstimator.setPose(
          new Pose2d(getPosition().getTranslation(), Constants.INIT_POSE.getRotation()));
      Overrides.resetYaw.set(false);
    }
  }

  private SwerveModuleState[] getWheelSpeeds() {
    return new SwerveModuleState[] {
      fl.getChassisRelativeState(),
      fr.getChassisRelativeState(),
      bl.getChassisRelativeState(),
      br.getChassisRelativeState()
    };
  }

  public Pose2d getPosition() {
    return poseEstimator.getPose();
  }

  private void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_WHEEL_SPEED);
    Logger.recordOutput("Drive/CmdStates", states);
    fl.setState(states[0]);
    fr.setState(states[1]);
    bl.setState(states[2]);
    br.setState(states[3]);
  }

  public void setPosition(Pose2d pos) {
    poseEstimator.setPose(pos);
  }

  public void addVisionUpdate(Pose2d newPose, Vector<N3> stdDevs, double timestamp) {}

  public void humanDrive(ChassisSpeeds cmd) {
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(cmd);
    setModuleStates(states);
  }

  public Rotation3d getGyro() {
    return imuInputs.rotation;
  }

  public double[] getAcceleration() {
    return new double[] {imuInputs.xAccelMPS, imuInputs.yAccelMPS, imuInputs.zAccelMPS};
  }

  public void lockWheels() {
    setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }

  public ChassisSpeeds getVelocity() {
    return KINEMATICS.toChassisSpeeds(getWheelSpeeds());
  }

  public PoseEstimator getPE() {
    return poseEstimator;
  }

  PIDController xPid = new PIDController(1.0, 0.0, 0.0);
  PIDController yPid = new PIDController(1.0, 0.0, 0.0);
  PIDController thPid = new PIDController(1.0, 0.0, 0.0);

  public void followTrajectory(SwerveSample sample) {
    humanDrive(
        new ChassisSpeeds(
            sample.vx, sample.vy, sample.omega)); // TODO: probably use PID or something
  }
}
