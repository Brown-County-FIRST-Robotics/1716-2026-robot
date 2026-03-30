package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.OurConstants;

public class RollersIOKraken implements RollersIO {
  TalonFX rollerMotor;
  TalonFX kickerMotor;
  TalonFX kicklerMotor;
  StatusSignal<AngularVelocity> rollerVelocity;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<Current> rollerCurrent;
  StatusSignal<AngularVelocity> kickerVelocity;
  StatusSignal<Voltage> kickerAppliedVolts;
  StatusSignal<Current> kickerCurrent;
  StatusSignal<AngularVelocity> kicklerVelocity;
  StatusSignal<Voltage> kicklerAppliedVolts;
  StatusSignal<Current> kicklerCurrent;

  public RollersIOKraken(int kicklerID, int rollerID, int kickerID) {
    kicklerMotor = new TalonFX(kicklerID, OurConstants.CAN_BUS);
    rollerMotor = new TalonFX(rollerID, OurConstants.CAN_BUS);
    kickerMotor = new TalonFX(kickerID, OurConstants.CAN_BUS);

    rollerVelocity = rollerMotor.getVelocity();
    rollerCurrent = rollerMotor.getStatorCurrent();
    rollerCurrent.setUpdateFrequency(50);
    rollerAppliedVolts = rollerMotor.getMotorVoltage();

    kickerVelocity = kickerMotor.getVelocity();
    kickerCurrent = kickerMotor.getStatorCurrent();
    kickerCurrent.setUpdateFrequency(50);
    kickerAppliedVolts = kickerMotor.getMotorVoltage();

    kicklerVelocity = kicklerMotor.getVelocity();
    kicklerAppliedVolts = kicklerMotor.getMotorVoltage();
    kicklerCurrent = kicklerMotor.getStatorCurrent();
    kicklerCurrent.setUpdateFrequency(50);

    var kickler_cfg = new TalonFXConfiguration();
    kickler_cfg.Slot0 = new Slot0Configs().withKV(12.0 / 100.0).withKP(12.0 / 100.0);
    kickler_cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kickler_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    kicklerMotor.getConfigurator().apply(kickler_cfg);

    var kicker_cfgr = kickerMotor.getConfigurator();
    kicker_cfgr.apply(
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5)
            .withTorqueClosedLoopRampPeriod(0.5));
    // Using KS because of our friction-based belt tensioning system
    kicker_cfgr.apply(
        new Slot0Configs()
            .withKP(2 * 12.0 / 120.0)
            .withKI(0)
            .withKD(0)
            .withKV(12.0 / 120.0)
            .withKA(0)
            .withKS(0.8));
    var roller_cfgr = rollerMotor.getConfigurator();
    roller_cfgr.apply(
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5)
            .withTorqueClosedLoopRampPeriod(0.5));
    roller_cfgr.apply(
        new Slot0Configs()
            .withKP(2 * 12.0 / 120.0)
            .withKI(0)
            .withKD(0)
            .withKV(12.0 / 120.0)
            .withKA(0)
            .withKS(0));
    roller_cfgr.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    var rollerStatus =
        BaseStatusSignal.refreshAll(rollerCurrent, rollerAppliedVolts, rollerVelocity);
    var kickerStatus =
        BaseStatusSignal.refreshAll(kickerCurrent, kickerAppliedVolts, kickerVelocity);
    var kicklerStatus =
        BaseStatusSignal.refreshAll(kicklerCurrent, kicklerAppliedVolts, kickerVelocity);

    inputs.rollerVelocity = rollerVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.rollersConnected = rollerStatus.isOK();
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerAppliedCurrent = rollerCurrent.getValueAsDouble();

    inputs.kickerVelocity = kickerVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.kickerConnected = kickerStatus.isOK();
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerAppliedCurrent = kickerCurrent.getValueAsDouble();

    inputs.kicklerVelocity = kicklerVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.kicklerConnected = kicklerStatus.isOK();
    inputs.kicklerAppliedVolts = kicklerAppliedVolts.getValueAsDouble();
    inputs.kicklerAppliedCurrent = kicklerCurrent.getValueAsDouble();
  }

  @Override
  public void commandSpeed(
      double kickler_velocity, double roller_velocity, double kicker_velocity) {
    kicklerMotor.setControl(new VelocityVoltage(kicker_velocity));
    rollerMotor.setControl(new VelocityVoltage(roller_velocity));
    kickerMotor.setControl(new VelocityVoltage(kicker_velocity));
  }
}
