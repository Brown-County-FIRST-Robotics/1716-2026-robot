package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.OurConstants;

public class IntakeIOKraken implements IntakeIO {
  TalonFX intakeMotor;
  TalonFX extendMotor;
  StatusSignal<AngularVelocity> intakeVelocity;
  StatusSignal<Current> intakeCurrent;
  StatusSignal<Voltage> intakeAppliedVolts;
  StatusSignal<AngularVelocity> extendVelocity;
  StatusSignal<Current> extendCurrent;
  StatusSignal<Voltage> extendAppliedVolts;
  TrapezoidProfile.State cstate;
  TrapezoidProfile.State commState;
  TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(10, 10));

  public IntakeIOKraken(int intakeID, int extendID) {
    intakeMotor = new TalonFX(intakeID, OurConstants.INTAKE_CAN_BUS);
    extendMotor = new TalonFX(extendID, OurConstants.CAN_BUS);
    intakeVelocity = intakeMotor.getVelocity();
    intakeCurrent = intakeMotor.getStatorCurrent();
    intakeCurrent.setUpdateFrequency(50);
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    extendVelocity = extendMotor.getVelocity();
    extendCurrent = extendMotor.getStatorCurrent();
    extendCurrent.setUpdateFrequency(50);
    extendAppliedVolts = extendMotor.getMotorVoltage();
    var extend_cfgr = extendMotor.getConfigurator();

    extend_cfgr.apply(new Slot0Configs().withKP(0.5).withKV(0.12).withKS(1));
    extend_cfgr.apply(new Slot1Configs().withKP(0.25).withKV(0.06).withKS(1));
    extend_cfgr.apply(new Slot2Configs().withKP(0.15).withKV(0.3).withKS(1));
    extend_cfgr.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    var intake_cfgr = intakeMotor.getConfigurator();
    intake_cfgr.apply(
        new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5)
            .withTorqueClosedLoopRampPeriod(0.5));
    intake_cfgr.apply(
        new Slot0Configs().withKV(12.0 / (7758.0 / 60.0)).withKP(0.8 * 12.0 / (7758.0 / 60.0)));
    intake_cfgr.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    // Motion magic control for extension/retraction
    var mmc = new TalonFXConfiguration().MotionMagic;
    mmc.MotionMagicCruiseVelocity = 8;
    mmc.MotionMagicAcceleration = 3;
    mmc.MotionMagicJerk = 30;

    extend_cfgr.apply(mmc);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus =
        BaseStatusSignal.refreshAll(intakeCurrent, intakeAppliedVolts, intakeVelocity);
    var extendStatus =
        BaseStatusSignal.refreshAll(extendCurrent, extendAppliedVolts, extendVelocity);
    inputs.extendConnected = extendStatus.isOK();
    inputs.intakeConnected = intakeStatus.isOK();
    inputs.extendAppliedVolts = extendAppliedVolts.getValueAsDouble();
    inputs.extendVelocity = extendVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.extendAppliedCurrent = extendCurrent.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeVelocity = intakeVelocity.getValue().in(Units.RotationsPerSecond);
    inputs.intakeAppliedCurrent = intakeCurrent.getValueAsDouble();
  }

  @Override
  public void intakeSpeed(double intake_vel) {
    intakeMotor.setControl(new VelocityVoltage(intake_vel));
  }

  @Override
  public void extenderPosition(double extendPosition) {
    // commState = new TrapezoidProfile.State(extendPosition, 0);

    var request = new MotionMagicVoltage(0);
    extendMotor.setControl(request.withPosition(extendPosition));

    // profile.calculate(0.02, cstate, commState);
    // extendMotor.setControl(new PositionVoltage(extendPosition));
  }

  @Override
  public void extenderVelocity(double rps) {
    // Negative rps means retracting
    if (rps == 0) extendMotor.setControl(new VelocityVoltage(rps).withSlot(2));
    else if (rps < 0) extendMotor.setControl(new VelocityVoltage(rps).withSlot(0));
    else extendMotor.setControl(new VelocityVoltage(rps).withSlot(1));
  }
}
