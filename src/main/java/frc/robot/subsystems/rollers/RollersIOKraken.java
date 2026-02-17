package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;

public class RollersIOKraken {
  TalonFX motor;
  StatusSignal<AngularVelocity> velocity;
  Servo serv;

  public RollersIOKraken(CANBus cbus, int id) {
    motor = new TalonFX(id, cbus);
  }
}
