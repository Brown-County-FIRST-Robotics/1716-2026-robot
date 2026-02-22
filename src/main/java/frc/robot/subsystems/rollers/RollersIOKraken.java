package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RollersIOKraken implements RollersIO {
  TalonFX rollerMotor;
  TalonFX kickerMotor;
  StatusSignal<AngularVelocity> rollerVelocity;
  StatusSignal<Current> rollerCurrent;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<AngularVelocity> kickerVelocity;
  StatusSignal<Current> kickerCurrent;
  StatusSignal<Voltage> kickerAppliedVolts;

  public RollersIOKraken(CANBus canbus, int rollerID, int kickerID) {
    // rollerMotor = new TalonFX(rollerID, canbus);
    kickerMotor = new TalonFX(kickerID, canbus);
    // rollerVelocity = rollerMotor.getVelocity();
    // rollerCurrent = rollerMotor.getStatorCurrent();
    // rollerCurrent.setUpdateFrequency(50);
    //  rollerAppliedVolts = rollerMotor.getMotorVoltage();
    kickerVelocity = kickerMotor.getVelocity();
    kickerCurrent = kickerMotor.getStatorCurrent();
    kickerCurrent.setUpdateFrequency(50);
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    var kicker_cfgr = kickerMotor.getConfigurator();
    kicker_cfgr.apply(
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5)
            .withTorqueClosedLoopRampPeriod(0.5));
    kicker_cfgr.apply(
        new Slot0Configs().withKV(12.0 / (7758.0 / 60.0)).withKP(1.2 * 12.0 / (7758.0 / 60.0)));
    // var roller_cfgr = rollerMotor.getConfigurator();
    // roller_cfgr.apply(
    //  new ClosedLoopRampsConfigs()
    //    .withDutyCycleClosedLoopRampPeriod(0.5)
    //  .withTorqueClosedLoopRampPeriod(0.5));
    // roller_cfgr.apply(
    //  new Slot0Configs().withKV(12.0 / (7758.0 / 60.0)).withKP(0.8 * 12.0 / (7758.0 / 60.0)));
  }

  @Override
  public void justSet(double volt) {
    rollerMotor.setControl(new VoltageOut(volt));
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    //    var rollerStatus =
    //        BaseStatusSignal.refreshAll(rollerCurrent, rollerAppliedVolts, rollerVelocity);
    var kickerStatus =
        BaseStatusSignal.refreshAll(kickerCurrent, kickerAppliedVolts, kickerVelocity);
    inputs.kickerConnected = kickerStatus.isOK();
    //    inputs.rollersConnected = rollerStatus.isOK();
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerVelocity = kickerVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.kickerAppliedCurrent = kickerCurrent.getValueAsDouble();
    //    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    //    inputs.rollerVelocity = rollerVelocity.getValue().in(Units.RotationsPerSecond);
    //    inputs.rollerAppliedCurrent = rollerCurrent.getValueAsDouble();
  }

  @Override
  public void commandSpeed(double rollerVelocity, double kicker_velocity) {
    kickerMotor.setControl(new VelocityVoltage(kicker_velocity));
    //  rollerMotor.setControl(new VelocityVoltage(rollerVelocity));
  }
}
