package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;

public class ShooterIOKrakens implements ShooterIO {

  TalonFX shooterMotor;
  Servo hoodServo;

  StatusSignal<AngularVelocity> shooterVelocitySignal;
  StatusSignal<Current> shooterCurrent;
  StatusSignal<Voltage> shooterAppliedOutputs;

  public ShooterIOKrakens(int motorID, int servoID) {
    shooterMotor = new TalonFX(motorID, new CANBus("1716_canivore"));
    hoodServo = new Servo(servoID);

    var configurator = shooterMotor.getConfigurator();
    configurator.apply(new Slot0Configs().withKP(1).withKV(1).withKI(0).withKD(0).withKA(0));
    configurator.apply(new ClosedLoopGeneralConfigs());

    shooterVelocitySignal = shooterMotor.getVelocity();
    shooterAppliedOutputs = shooterMotor.getMotorVoltage();
    shooterCurrent = shooterMotor.getStatorCurrent();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(shooterAppliedOutputs, shooterVelocitySignal, shooterCurrent);
    inputs.shooterAppliedOutput = shooterAppliedOutputs.getValueAsDouble();
    inputs.shooterCurrent = shooterCurrent.getValueAsDouble();
    inputs.shooterHoodPosition = hoodServo.getPosition();
    inputs.shooterVelocity = shooterVelocitySignal.getValueAsDouble();
    inputs.connected = status.isOK();
  }

  @Override
  public void commandShooterSpeed(double speed) {
    shooterMotor.setControl(new VelocityVoltage(speed));
  }

  @Override
  public void commandHoodPosition(double length) {
    hoodServo.setPulseTimeMicroseconds((int) (1000 + 1000 * (length / 2)));
  }
}
