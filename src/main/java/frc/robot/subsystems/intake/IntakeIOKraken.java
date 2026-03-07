package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOKraken implements IntakeIO {
  TalonFX intakeMotor;
  TalonFX extendMotor;
  StatusSignal<AngularVelocity> intakeVelocity;
  StatusSignal<Current> intakeCurrent;
  StatusSignal<Voltage> intakeAppliedVolts;
  StatusSignal<AngularVelocity> extendVelocity;
  StatusSignal<Current> extendCurrent;
  StatusSignal<Voltage> extendAppliedVolts;

  public IntakeIOKraken(CANBus canbus, int intakeID, int extendID) {
    intakeMotor = new TalonFX(intakeID, canbus);
    // extendMotor = new TalonFX(extendID, canbus);
    intakeVelocity = intakeMotor.getVelocity();
    intakeCurrent = intakeMotor.getStatorCurrent();
    intakeCurrent.setUpdateFrequency(50);
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    // extendVelocity = extendMotor.getVelocity();
    // extendCurrent = extendMotor.getStatorCurrent();
    // extendCurrent.setUpdateFrequency(50);
    // extendAppliedVolts = extendMotor.getMotorVoltage();
    // var extend_cfgr = extendMotor.getConfigurator();
    // extend_cfgr.apply(
    //     new ClosedLoopRampsConfigs()
    //         .withDutyCycleClosedLoopRampPeriod(0.5)
    //         .withTorqueClosedLoopRampPeriod(0.5));
    // extend_cfgr.apply(
    //     new Slot0Configs().withKV(12.0 / (7758.0 / 60.0)).withKP(1.2 * 12.0 / (7758.0 / 60.0)));
    var intake_cfgr = intakeMotor.getConfigurator();
    intake_cfgr.apply(
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5)
            .withTorqueClosedLoopRampPeriod(0.5));
    intake_cfgr.apply(
        new Slot0Configs().withKV(12.0 / (7758.0 / 60.0)).withKP(0.8 * 12.0 / (7758.0 / 60.0)));
    intake_cfgr.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus =
        BaseStatusSignal.refreshAll(intakeCurrent, intakeAppliedVolts, intakeVelocity);
    // var extendStatus =
    //     BaseStatusSignal.refreshAll(extendCurrent, extendAppliedVolts, extendVelocity);
    // inputs.extendConnected = extendStatus.isOK();
    // inputs.extendConnected = intakeStatus.isOK();
    // inputs.extendAppliedVolts = extendAppliedVolts.getValueAsDouble();
    // inputs.extendVelocity = extendVelocity.getValue().in(Units.RotationsPerSecond);
    // inputs.extendAppliedCurrent = extendCurrent.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeVelocity = intakeVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.intakeAppliedCurrent = intakeCurrent.getValueAsDouble();
  }

  @Override
  public void intakeSpeed(double intake_vel) {
    intakeMotor.setControl(new VelocityVoltage(intake_vel));
  }

  @Override
  public void extendSpeed(double extend_vel) {
    // extendMotor.setControl(new VelocityVoltage(extend_vel));
  }
}
