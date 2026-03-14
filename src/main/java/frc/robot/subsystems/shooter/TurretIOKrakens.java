package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.OurConstants;

public class TurretIOKrakens implements TurretIO {
  TalonFX motor;
  CANcoder encoderA;
  CANcoder encoderB;
  StatusSignal<Angle> a_angle;
  StatusSignal<Angle> b_angle;

  public TurretIOKrakens(int motorID, int encoder_a_id, int encoder_b_id) {
    motor = new TalonFX(motorID, OurConstants.CAN_BUS);
    encoderA = new CANcoder(encoder_a_id, OurConstants.CAN_BUS);
    encoderB = new CANcoder(encoder_b_id, OurConstants.CAN_BUS);
    a_angle = encoderA.getAbsolutePosition();
    b_angle = encoderB.getAbsolutePosition();
  }

  @Override
  public void commandPosition(double position) {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.encoder_a_position = a_angle.getValue().in(Units.Rotations);
    inputs.encoder_b_position = b_angle.getValue().in(Units.Rotations);
  }
}
