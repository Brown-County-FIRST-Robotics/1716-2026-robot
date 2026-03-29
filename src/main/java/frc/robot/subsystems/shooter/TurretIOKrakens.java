package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.OurConstants;
import org.littletonrobotics.junction.Logger;

public class TurretIOKrakens implements TurretIO {
  TalonFX motor;
  CANcoder encoderA;
  CANcoder encoderB;
  StatusSignal<Angle> a_angle;
  StatusSignal<Angle> b_angle;
  StatusSignal<Angle> turretPosition;
  StatusSignal<AngularVelocity> turretVelocity;
  StatusSignal<Current> turretCurrent;
  StatusSignal<Voltage> turretAppliedOutput;

  public TurretIOKrakens(int motorID, int encoder_a_id, int encoder_b_id) {
    motor = new TalonFX(motorID, OurConstants.CAN_BUS);
    encoderA = new CANcoder(encoder_a_id, OurConstants.CAN_BUS);
    encoderB = new CANcoder(encoder_b_id, OurConstants.CAN_BUS);
    a_angle = encoderA.getAbsolutePosition();
    b_angle = encoderB.getAbsolutePosition();
    turretPosition = motor.getPosition();
    turretVelocity = motor.getVelocity();
    turretCurrent = motor.getStatorCurrent();
    turretAppliedOutput = motor.getMotorVoltage();

    var config = new TalonFXConfiguration();
    config.Slot0 = new Slot0Configs().withKV(12.0 / 100.0).withKP(12.0 / 100.0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var mmc = config.MotionMagic;

    mmc.MotionMagicCruiseVelocity = 50;
    mmc.MotionMagicAcceleration = 100;
    mmc.MotionMagicJerk = 1000;

    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void commandPosition(double position) {
    Logger.recordOutput("turret/realCommandPosition", position);
    var request = new MotionMagicVoltage(0);
    motor.setControl(request.withPosition(position * 480.0 / 10.0));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        a_angle, b_angle, turretCurrent, turretAppliedOutput, turretPosition, turretVelocity);

    inputs.velocity = turretVelocity.getValueAsDouble();
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
    inputs.position = turretPosition.getValue().in(Units.Rotations) * 10.0 / 480.0;
    inputs.current = turretCurrent.getValueAsDouble();
    
    inputs.encoder_b_position = b_angle.getValue().in(Units.Rotations) + 0.363;
    inputs.encoder_a_position = a_angle.getValue().in(Units.Rotations) - 0.023;

    inputs.motorConnected = motor.isConnected();
    inputs.encoderAConnected = encoderA.isConnected();
    inputs.encoderBConnected = encoderB.isConnected();

    inputs.appliedVolts = turretAppliedOutput.getValueAsDouble();
  }
}
