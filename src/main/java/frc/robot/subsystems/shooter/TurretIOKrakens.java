package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.OurConstants;

public class TurretIOKrakens implements TurretIO {
  TalonFX motor;
  CANcoder encoderA;
  CANcoder encoderB;
  StatusSignal<Angle> a_angle;
  StatusSignal<Angle> b_angle;
  StatusSignal<Current> turretCurrent;
  StatusSignal<Voltage> turretAppliedOutput;

  public TurretIOKrakens(int motorID, int encoder_a_id, int encoder_b_id) {
    motor = new TalonFX(motorID, OurConstants.CAN_BUS);
    encoderA = new CANcoder(encoder_a_id, OurConstants.CAN_BUS);
    encoderB = new CANcoder(encoder_b_id, OurConstants.CAN_BUS);
    a_angle = encoderA.getAbsolutePosition();
    b_angle = encoderB.getAbsolutePosition();
    motor.getPosition();
  }

  @Override
  public void commandPosition(double position) {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(a_angle, b_angle);
    inputs.encoder_a_position = a_angle.getValue().in(Units.Rotations) - 0.023;
    inputs.encoder_b_position = b_angle.getValue().in(Units.Rotations) + 0.363;
  }
}
