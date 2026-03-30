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
  TalonFX starsMotor;
  TalonFX kickerMotor;
  TalonFX beltsMotor;
  StatusSignal<AngularVelocity> starsVelocity;
  StatusSignal<Voltage> starsAppliedVolts;
  StatusSignal<Current> starsCurrent;
  StatusSignal<AngularVelocity> kickerVelocity;
  StatusSignal<Voltage> kickerAppliedVolts;
  StatusSignal<Current> kickerCurrent;
  StatusSignal<AngularVelocity> beltsVelocity;
  StatusSignal<Voltage> beltsAppliedVolts;
  StatusSignal<Current> beltsCurrent;

  public RollersIOKraken(int beltsID, int starsID, int kickerID) {
    beltsMotor = new TalonFX(beltsID, OurConstants.CAN_BUS);
    starsMotor = new TalonFX(starsID, OurConstants.CAN_BUS);
    kickerMotor = new TalonFX(kickerID, OurConstants.CAN_BUS);

    starsVelocity = starsMotor.getVelocity();
    starsAppliedVolts = starsMotor.getMotorVoltage();
    starsCurrent = starsMotor.getStatorCurrent();
    starsCurrent.setUpdateFrequency(50);

    kickerVelocity = kickerMotor.getVelocity();
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    kickerCurrent = kickerMotor.getStatorCurrent();
    kickerCurrent.setUpdateFrequency(50);

    beltsVelocity = beltsMotor.getVelocity();
    beltsAppliedVolts = beltsMotor.getMotorVoltage();
    beltsCurrent = beltsMotor.getStatorCurrent();
    beltsCurrent.setUpdateFrequency(50);

    var belts_cfg = new TalonFXConfiguration();
    belts_cfg.Slot0 = new Slot0Configs().withKV(12.0 / 100.0).withKP(12.0 / 100.0);
    belts_cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    belts_cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    beltsMotor.getConfigurator().apply(belts_cfg);

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
    var stars_cfgr = starsMotor.getConfigurator();
    stars_cfgr.apply(
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5)
            .withTorqueClosedLoopRampPeriod(0.5));
    stars_cfgr.apply(
        new Slot0Configs()
            .withKP(2 * 12.0 / 120.0)
            .withKI(0)
            .withKD(0)
            .withKV(12.0 / 120.0)
            .withKA(0)
            .withKS(0));
    stars_cfgr.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    var starsStatus = BaseStatusSignal.refreshAll(starsCurrent, starsAppliedVolts, starsVelocity);
    var kickerStatus =
        BaseStatusSignal.refreshAll(kickerCurrent, kickerAppliedVolts, kickerVelocity);
    var beltsStatus = BaseStatusSignal.refreshAll(beltsCurrent, beltsAppliedVolts, beltsVelocity);

    inputs.starsVelocity = starsVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.starsConnected = starsStatus.isOK();
    inputs.starsAppliedVolts = starsAppliedVolts.getValueAsDouble();
    inputs.starsAppliedCurrent = starsCurrent.getValueAsDouble();

    inputs.kickerVelocity = kickerVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.kickerConnected = kickerStatus.isOK();
    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerAppliedCurrent = kickerCurrent.getValueAsDouble();

    inputs.beltsVelocity = beltsVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.beltsConnected = beltsStatus.isOK();
    inputs.beltsAppliedVolts = beltsAppliedVolts.getValueAsDouble();
    inputs.beltsAppliedCurrent = beltsCurrent.getValueAsDouble();
  }

  @Override
  public void commandSpeed(double belts_velocity, double stars_velocity, double kicker_velocity) {
    beltsMotor.setControl(new VelocityVoltage(belts_velocity));
    starsMotor.setControl(new VelocityVoltage(stars_velocity));
    kickerMotor.setControl(new VelocityVoltage(kicker_velocity));
  }
}
