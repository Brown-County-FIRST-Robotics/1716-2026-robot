package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

public class ModuleIOKrakens implements ModuleIO {
  TalonFX steer;
  private final double THRUST_DISTANCE_PER_TICK = .0254 * 4.0 * Math.PI / 5.36;

  TalonFX thrust;
  CANcoder steercoder;

  StatusSignal<Angle> thrust_pos_signal;
  StatusSignal<Angle> steer_angle_signal;
  double off;

  public ModuleIOKrakens(int steerid, int thrustid, int coder_id, String name) {
    steer = new TalonFX(steerid, new CANBus("1716_canivore"));
    thrust = new TalonFX(thrustid, new CANBus("1716_canivore"));
    steercoder = new CANcoder(coder_id, new CANBus("1716_canivore"));
    var cfgg = steer.getConfigurator();

    TalonFXConfiguration steerconf = new TalonFXConfiguration();
    TalonFXConfiguration thrustconf = new TalonFXConfiguration();
    thrustconf.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    thrustconf.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    thrustconf.Slot0.withKP(0).withKI(0).withKD(0).withKV(12.0 / 100.0);

    steerconf.Feedback.withFusedCANcoder(steercoder);
    steerconf.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    steerconf.Feedback.withRotorToSensorRatio(150.0 / 7.0);
    steerconf.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    steerconf.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    steerconf.Slot0.withKP(10).withKI(0).withKD(0).withKV((150.0 / 7.0) * 12.0 / 100.0);
    steerconf.MotorOutput.DutyCycleNeutralDeadband = 0.01;
    steerconf.ClosedLoopGeneral.withContinuousWrap(true);

    cfgg.apply(steerconf);
    thrust.getConfigurator().apply(thrustconf);

    Logger.recordOutput("Firmware/" + name + "_Steer", steer.getVersion().getValue());
    Logger.recordOutput("Firmware/" + name + "_Thrust", thrust.getVersion().getValue());
    thrust_pos_signal = thrust.getPosition();
    steer_angle_signal = steercoder.getAbsolutePosition();
    if (thrustid == 5) {
      off = 0.491; // FL
    } else if (thrustid == 8) {
      off = 0.241; // FR
    } else if (thrustid == 3) {
      off = -0.245 + 0.5; // BL
    } else if (thrustid == 2) {
      off = -0.496; // BR
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.offset = off;

    BaseStatusSignal.refreshAll(thrust_pos_signal, steer_angle_signal);
    inputs.thrustPos = thrust_pos_signal.getValue().in(Units.Rotations) * THRUST_DISTANCE_PER_TICK;
    inputs.absSensorAngle = steer_angle_signal.getValue().in(Units.Rotations);
  }

  @Override
  public void setCmdState(double ang, double vel) {
    thrust.setControl(new VelocityVoltage(vel / THRUST_DISTANCE_PER_TICK));
    steer.setControl(new PositionVoltage(ang));
  }
}
