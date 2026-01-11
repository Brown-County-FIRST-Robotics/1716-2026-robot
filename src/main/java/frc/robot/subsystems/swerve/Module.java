package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** A class representing a single Swerve module */
public class Module {
  private static final LoggedTunableNumber minNoMotionTime =
      new LoggedTunableNumber("Min no motion time", 5);
  private static final LoggedTunableNumber maxMotionAllowed =
      new LoggedTunableNumber("Max motion", 0.005);
  final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  final ModuleIO io;
  final int ind;
  String name;
  Rotation2d chassisOffset;
  Rotation2d relativeSensorZeroPosition = new Rotation2d();
  final Timer noMotionTimer = new Timer();

  public static class CANAddress { // Forward compatibility with 2027
    public int id;

    public enum Bus {
      CANIVORE,
      CAN_0,
      CAN_1,
      CAN_2,
      CAN_3,
      CAN_4
    }

    public Bus bus;

    public CANAddress(int id, Bus bus) {
      this.id = id;
      this.bus = bus;
    }

    public CANAddress rio(int id) {
      return new CANAddress(id, Bus.CAN_0);
    }

    public CANAddress canivore(int id) {
      return new CANAddress(id, Bus.CANIVORE);
    }
  }

  public static class Config {
    public CANAddress thrustMotor;
    public CANAddress steerMotor;

    private double thrust_KV;
    private double thrust_P;
    private double thrust_I;
    private double thrust_D;

    private double steer_KV;
    private double steer_P;
    private double steer_I;
    private double steer_D;

    private double steer_offset;
    private boolean flip_steer;
    private boolean flip_thrust;
  }

  /**
   * Creates a new Swerve Module
   *
   * @param io The IO for the module
   * @param ind The index of the module (fl:0, fr:1, bl:2, br:3)
   */
  public Module(ModuleIO io, int ind) {
    this.io = io;
    this.ind = ind;
    switch (ind) {
      case 0:
        chassisOffset = Rotation2d.fromDegrees(-90);
        name = "FL";
        break;
      case 1:
        chassisOffset = Rotation2d.fromDegrees(0);
        name = "FR";
        break;
      case 2:
        chassisOffset = Rotation2d.fromDegrees(180);
        name = "BL";
        break;
      case 3:
        chassisOffset = Rotation2d.fromDegrees(90);
        name = "BR";
        break;
    }

    CustomAlerts.makeOverTempAlert(() -> inputs.steerTempC, 60, 50, name + " steer motor");
    CustomAlerts.makeOverTempAlert(() -> inputs.thrustTempC, 80, 70, name + " thrust motor");

    periodic();
    reZero();
  }

  /** Periodic functionality. Call every tick. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name + "_Inputs", inputs);
    if (Math.abs(inputs.absSensorOmega) > maxMotionAllowed.get()
        || Math.abs(inputs.relativeSensorOmega) > maxMotionAllowed.get()) {
      noMotionTimer.restart();
    }
    if (noMotionTimer.hasElapsed(minNoMotionTime.get()) && DriverStation.isDisabled()) {
      reZero();
    }
  }

  private void reZero() {
    Logger.recordOutput(
        "Drive/" + name + "/relative_encoder_drift",
        getAbsEncoderPos().minus(getProcessedRelativeEncoderPos()).getDegrees());
    relativeSensorZeroPosition = getAbsEncoderPos().minus(getUnprocessedRelativeEncoderPos());
    noMotionTimer.restart();
  }

  private Rotation2d getProcessedRelativeEncoderPos() {
    return relativeSensorZeroPosition.plus(getUnprocessedRelativeEncoderPos());
  }

  private Rotation2d getUnprocessedRelativeEncoderPos() {
    return Rotation2d.fromRotations(inputs.relativeSensorAngle);
  }

  private Rotation2d getAbsEncoderPos() {
    return Rotation2d.fromRotations(inputs.absSensorAngle)
        .minus(Rotation2d.fromRotations(inputs.offset));
  }

  private Rotation2d getChassisRelativeRotation() {
    return getProcessedRelativeEncoderPos().minus(chassisOffset);
  }

  /**
   * Gets the module state
   *
   * @return The module state
   */
  public SwerveModuleState getChassisRelativeState() {
    return new SwerveModuleState(inputs.thrustVel, getChassisRelativeRotation());
  }

  /**
   * Gets the module position
   *
   * @return The module position
   */
  public SwerveModulePosition getChassisRelativePosition() {
    return new SwerveModulePosition(inputs.thrustPos, getChassisRelativeRotation());
  }

  /**
   * Commands a state to the module
   *
   * @param state The command state
   */
  public void setState(SwerveModuleState state) {
    state.optimize(getChassisRelativeRotation());
    state.speedMetersPerSecond *= getChassisRelativeRotation().minus(state.angle).getCos();
    Rotation2d cmdPosForRelativeEncoder =
        state.angle.plus(chassisOffset).minus(relativeSensorZeroPosition);
    double adjustedRelCmd =
        inputs.relativeSensorAngle
            - (inputs.relativeSensorAngle % 1.0)
            + cmdPosForRelativeEncoder.getRotations();
    if (Math.abs(adjustedRelCmd - inputs.relativeSensorAngle)
        > Math.abs(1.0 + adjustedRelCmd - inputs.relativeSensorAngle)) {
      adjustedRelCmd += 1.0;
    }
    if (Math.abs(adjustedRelCmd - inputs.relativeSensorAngle)
        > Math.abs(-1.0 + adjustedRelCmd - inputs.relativeSensorAngle)) {
      adjustedRelCmd -= 1.0;
    }

    io.setCmdState(adjustedRelCmd, state.speedMetersPerSecond);
  }
}
