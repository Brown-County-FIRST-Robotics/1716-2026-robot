package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

/** A class representing a single Swerve module */
public class Module {
  final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  final ModuleIO io;
  final int ind;
  String name;
  Rotation2d chassisOffset;
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
  }

  /** Periodic functionality. Call every tick. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + name + "_Inputs", inputs);
  }

  private Rotation2d getEncoderPos() {
    return Rotation2d.fromRotations(inputs.absSensorAngle)
        .minus(Rotation2d.fromRotations(inputs.offset));
  }

  private Rotation2d getChassisRelativeRotation() {
    return getEncoderPos().minus(chassisOffset);
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
    System.out.println(state);
    state.optimize(getChassisRelativeRotation());
    state.speedMetersPerSecond *= getChassisRelativeRotation().minus(state.angle).getCos();
    Rotation2d cmdPosForRelativeEncoder =
        state.angle.plus(chassisOffset).plus(Rotation2d.fromRotations(inputs.offset));
    Logger.recordOutput("SMOUT_" + name, cmdPosForRelativeEncoder);

    io.setCmdState(cmdPosForRelativeEncoder.getRotations(), state.speedMetersPerSecond);
  }
}
