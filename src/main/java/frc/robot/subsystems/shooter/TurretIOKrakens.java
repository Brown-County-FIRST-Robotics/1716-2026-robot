package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
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
  StatusSignal<Current> turretCurrent;
  StatusSignal<Voltage> turretAppliedOutput;

  public TurretIOKrakens(int motorID, int encoder_a_id, int encoder_b_id) {
    motor = new TalonFX(motorID, OurConstants.CAN_BUS);
    encoderA = new CANcoder(encoder_a_id, OurConstants.CAN_BUS);
    encoderB = new CANcoder(encoder_b_id, OurConstants.CAN_BUS);
    a_angle = encoderA.getAbsolutePosition();
    b_angle = encoderB.getAbsolutePosition();
    turretPosition = motor.getPosition();
    turretCurrent = motor.getStatorCurrent();
    turretAppliedOutput = motor.getMotorVoltage();

    var config = new TalonFXConfiguration();
    config.Slot0 = new Slot0Configs().withKV(6.0 * 12.0 / 100.0).withKP(4.0 * 12.0 / 100.0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // var mmc = config.MotionMagic;

    // mmc.MotionMagicCruiseVelocity = 100;
    // mmc.MotionMagicAcceleration = 1000;
    // mmc.MotionMagicJerk = 10000;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void commandPosition(double position) {
    Logger.recordOutput("turret/realCommandPosition", position);
    motor.setControl(new PositionVoltage(position * 480.0 / 10.0));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        a_angle, b_angle, turretCurrent, turretAppliedOutput, turretPosition);
    inputs.velocity = inputs.encoder_a_position = a_angle.getValue().in(Units.Rotations) - 0.023;
    inputs.encoder_b_position = b_angle.getValue().in(Units.Rotations) + 0.363;
    inputs.current = turretCurrent.getValueAsDouble();
    inputs.position = turretPosition.getValue().in(Units.Rotations) * 10.0 / 480.0;
    inputs.appliedVolts = turretAppliedOutput.getValueAsDouble();
  }
}
