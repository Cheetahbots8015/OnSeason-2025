// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.RollerConstants;
import frc.robot.util.LoggedTunableNumber;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class RollerSystemIOKrakenX60 implements RollerSystemIO {

  private final TalonFX talon;

  private TalonFXConfigurator Configurator;

  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final MotorOutputConfigs MotorConfigs;
  private final Slot0Configs slot0Configs;
  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Roller/kA", RollerConstants.ROLLER_KA);
  LoggedTunableNumber kS = new LoggedTunableNumber("Roller/kS", RollerConstants.ROLLER_KS);
  LoggedTunableNumber kV = new LoggedTunableNumber("Roller/kV", RollerConstants.ROLLER_KV);
  LoggedTunableNumber kP = new LoggedTunableNumber("Roller/kP", RollerConstants.ROLLER_KP);
  LoggedTunableNumber kI = new LoggedTunableNumber("Roller/kI", RollerConstants.ROLLER_KI);
  LoggedTunableNumber kD = new LoggedTunableNumber("Roller/kD", RollerConstants.ROLLER_KD);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  public RollerSystemIOKrakenX60() {

    /* Instantiate motors and configurators */
    this.talon = new TalonFX(RollerConstants.ROLLER_ID, "dabus");

    this.Configurator = talon.getConfigurator();

    /* Create configs */
    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable =
        RollerConstants.ROLLER_STATOR_CURRENT_LIMIT_ENABLE;
    currentLimitsConfigs.SupplyCurrentLimitEnable =
        RollerConstants.ROLLER_SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitsConfigs.StatorCurrentLimit = RollerConstants.ROLLER_STATOR_CURRENT_LIMIT_AMPS;
    currentLimitsConfigs.SupplyCurrentLimit = RollerConstants.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
    currentLimitsConfigs.SupplyCurrentLowerLimit =
        RollerConstants.ROLLER_SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    // if supply current limit is active for longer than this period, the supply
    // current will be adjusted to supply current lower limit
    currentLimitsConfigs.SupplyCurrentLowerTime = RollerConstants.ROLLER_SUPPLY_CURRENT_LOWER_TIME;

    MotorConfigs = new MotorOutputConfigs();
    MotorConfigs.Inverted =
        RollerConstants.ROLLER_INVERSION
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    MotorConfigs.PeakForwardDutyCycle = RollerConstants.ROLLER_PEAK_FORWARD_DUTY_CYCLE;
    MotorConfigs.PeakReverseDutyCycle = RollerConstants.ROLLER_PEAK_REVERSE_DUTY_CYCLE;
    MotorConfigs.NeutralMode =
        RollerConstants.ROLLER_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    slot0Configs = new Slot0Configs();
    slot0Configs.kA = kA.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod =
        RollerConstants.ROLLER_DUTYCYCLE_CLOSEDLOOP_RAMP_PERIOD;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod =
        RollerConstants.ROLLER_TORQUE_CLOSEDLOOP_RAMP_PERIOD;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod =
        RollerConstants.ROLLER_VOLTAGE_CLOSEDLOOP_RAMP_PERIOD;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod =
        RollerConstants.ROLLER_DUTYCYCLE_CLOSEDLOOP_RAMP_PERIOD;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod =
        RollerConstants.ROLLER_TORQUE_CLOSEDLOOP_RAMP_PERIOD;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        RollerConstants.ROLLER_VOLTAGE_CLOSEDLOOP_RAMP_PERIOD;

    /* Apply configs */
    tryUntilOk(5, () -> Configurator.apply(currentLimitsConfigs));
    tryUntilOk(5, () -> Configurator.apply(MotorConfigs));
    tryUntilOk(5, () -> Configurator.apply(slot0Configs));
    tryUntilOk(5, () -> Configurator.apply(openLoopRampsConfigs));
    tryUntilOk(5, () -> Configurator.apply(closedLoopRampsConfigs));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    acceleration = talon.getAcceleration();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                RollerConstants.SIGNAL_UPDATE_FREQUENCY_HZ,
                position,
                velocity,
                acceleration,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius));
    tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                acceleration,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.accelerationRadsPerSec2 = Units.rotationsToRadians(acceleration.getValueAsDouble());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void updateTunableNumbers() {
    if (kA.hasChanged(0)
        || kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      tryUntilOk(5, () -> Configurator.apply(slot0Configs));
    }
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrentVelocity(double velocity) {
    talon.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
