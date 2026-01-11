package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import frc.robot.utils.Alert;
import frc.robot.utils.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * IO layer for a SDS MK4i L2 swerve module using a Falcon 500 as thrust, and a Neo V1.1 as steering
 */
public class ModuleIOSparkFX implements ModuleIO {
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / 6.75;
  private static final double STEER_FREE_RPM = 5676.0;
  private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
  private final SparkMax steer;
  private final SparkAbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;
  private final SparkClosedLoopController pid;
  private final TalonFX thrust;
  final StatusSignal<AngularVelocity> velSignal;
  final StatusSignal<Angle> posSignal;
  final StatusSignal<Double> errSignal;
  final StatusSignal<Temperature> tempSignal;
  final StatusSignal<Double> outputSignal;
  double offset;

  final String name;
  final LoggedTunableNumber thrustP =
      new LoggedTunableNumber("Thrust P", 12.0 * 3.0 / (6380.0 / 60.0));
  final LoggedTunableNumber thrustI = new LoggedTunableNumber("Thrust I", 0);
  // TODO: add these back in
  LoggedTunableNumber thrustD = new LoggedTunableNumber("Thrust D", 0);
  final LoggedTunableNumber thrustKV = new LoggedTunableNumber("Thrust KV", 12.0 / 6380.0);
  final LoggedTunableNumber steerP = new LoggedTunableNumber("Steer P", 0);
  final LoggedTunableNumber steerI = new LoggedTunableNumber("Steer I", 0);
  final LoggedTunableNumber steerD = new LoggedTunableNumber("Steer D", 0);
  final LoggedTunableNumber steerKV = new LoggedTunableNumber("Steer KV", 1.0 / STEER_FREE_RPM);
  final LoggedTunableNumber offsetTun;
  double off;
  final SparkMaxConfig steerConfig = new SparkMaxConfig();

  /**
   * Makes a new instance using CAN IDs
   *
   * @param thrustID Thrust motor CAN ID
   * @param steerID Steer motor controller CAN ID
   * @param name The name of the module (e.g. "FL", "BR")
   */
  public ModuleIOSparkFX(int thrustID, int steerID, String name) {
    new Alert(
            "Using extremely deprecated code. Please don't expect this to work. ",
            Alert.AlertType.ERROR)
        .set(true);
    this.name = name;
    thrust = new TalonFX(thrustID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    thrust.getConfigurator().refresh(config);
    config.Audio.BeepOnConfig = false;
    config.Audio.BeepOnBoot = false;
    config.MotorOutput.PeakForwardDutyCycle = 1;
    config.MotorOutput.PeakReverseDutyCycle = -1;
    config.Audio.AllowMusicDurDisable = true;
    config.Slot0.kV = 12.0 * 60.0 / 6380.0;
    config.Slot0.kP = thrustP.get();
    config.Slot0.kI = thrustI.get();
    config.MotorOutput.DutyCycleNeutralDeadband = 0.01;
    offsetTun = new LoggedTunableNumber(name + "_offset");
    if (thrustID == 20) {
      off = 0.284; // BL
    } else if (thrustID == 21) {
      off = 0.01; // SPARE, FILL IN
    } else if (thrustID == 22) {
      off = 0.099; // BR
    } else if (thrustID == 23) {
      off = 0.995; // FR
    } else if (thrustID == 24) {
      off = 0.908; // FL
    }
    offsetTun.initDefault(off);
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    thrust.getConfigurator().apply(config);
    velSignal = thrust.getRotorVelocity();
    posSignal = thrust.getRotorPosition();
    errSignal = thrust.getClosedLoopError();
    tempSignal = thrust.getDeviceTemp();
    outputSignal = thrust.getClosedLoopOutput();
    velSignal.setUpdateFrequency(50.0);
    posSignal.setUpdateFrequency(50.0);
    errSignal.setUpdateFrequency(50.0);
    outputSignal.setUpdateFrequency(50.0);
    tempSignal.setUpdateFrequency(20.0);
    thrust.optimizeBusUtilization();
    steer = new SparkMax(steerID, SparkLowLevel.MotorType.kBrushless);
    steerConfig.encoder.positionConversionFactor(1);
    steerConfig.absoluteEncoder.inverted(false);
    steerP.attach(
        (Double p) -> {
          steerConfig.closedLoop.p(p);
          steer.configure(
              steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    steerI.attach(
        (Double p) -> {
          steerConfig.closedLoop.i(p);
          steer.configure(
              steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    steerD.attach(
        (Double p) -> {
          steerConfig.closedLoop.d(p);
          steer.configure(
              steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        });
    //    steerConfig
    //        .closedLoop
    //        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //        .p(0)
    //        .i(0)
    //        .d(0)
    //        .outputRange(-12, 12)
    //        .velocityFF(1.0 / 5676.0);
    //    steerConfig
    //        .closedLoop
    //        .smartMotion
    //        .maxVelocity(5767)
    //        .maxAcceleration(5000)
    //        .allowedClosedLoopError(0.0025);
    steerConfig.voltageCompensation(12.0);
    steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid = steer.getClosedLoopController();
    absoluteEncoder = steer.getAbsoluteEncoder();
    relativeEncoder = steer.getEncoder();
    steer.setInverted(true);

    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal, outputSignal);

    Logger.recordOutput("Firmware/" + name + "_Steer", steer.getFirmwareString());
    Logger.recordOutput("Firmware/" + name + "_Thrust", thrust.getVersion().getValue());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(velSignal, posSignal, errSignal, tempSignal, outputSignal);
    inputs.absSensorAngle = absoluteEncoder.getPosition();
    inputs.absSensorOmega = absoluteEncoder.getVelocity();
    inputs.relativeSensorAngle = relativeEncoder.getPosition() / STEER_GEAR_RATIO;
    inputs.relativeSensorOmega = relativeEncoder.getVelocity() / 60.0;
    inputs.thrustVel = velSignal.getValue().in(Units.RotationsPerSecond) * THRUST_DISTANCE_PER_TICK;
    inputs.thrustPos = posSignal.getValue().in(Units.Rotations) * THRUST_DISTANCE_PER_TICK;
    inputs.steerTempC = steer.getMotorTemperature();
    inputs.thrustErr = errSignal.getValue();
    inputs.thrustTempC = tempSignal.getValue().in(Units.Celsius);
    inputs.offset = offsetTun.get();
    inputs.thrustOutput = outputSignal.getValue();
  }

  @Override
  public void setCmdState(double ang, double vel) {
    thrust.setControl(new VelocityVoltage(vel / THRUST_DISTANCE_PER_TICK));
    //    pid.setReference(ang * STEER_GEAR_RATIO, ControlType.kSmartMotion, ClosedLoopSlot.kSlot0);
  }
}
