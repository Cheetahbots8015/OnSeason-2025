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
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RollerConstants;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class RollerSystemIOKrakenX60 implements RollerSystemIO {
  private final TalonFX talon;
  private final CANrange canRange;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  private final TalonFXConfiguration config;
  private final CANrangeConfiguration canConfig;

  public RollerSystemIOKrakenX60() {
    this.talon = new TalonFX(RollerConstants.ROLLER_ID, RollerConstants.ROLLER_CANNAME);
    this.canRange =
        new CANrange(RollerConstants.ROLLER_CANRANGE_ID, RollerConstants.ROLLER_CANNAME);

    this.config = new TalonFXConfiguration();
    this.canConfig = new CANrangeConfiguration();

    config.MotorOutput.withInverted(
        RollerConstants.ROLLER_INVERSION
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    config.Slot0.kP = RollerConstants.ROLLER_KP;
    config.Slot0.kI = RollerConstants.ROLLER_KI;
    config.Slot0.kD = RollerConstants.ROLLER_KD;
    config.Slot0.kA = RollerConstants.ROLLER_KA;
    config.Slot0.kS = RollerConstants.ROLLER_KS;
    config.Slot0.kV = RollerConstants.ROLLER_KV;
    talon.getConfigurator().apply(config);

    canConfig.ProximityParams.ProximityThreshold = RollerConstants.CANRANGE_DISTANCE_THRESHOLD;
    canConfig.ProximityParams.MinSignalStrengthForValidMeasurement =
        RollerConstants.MIN_SIGNAL_STRENGTH;
    canConfig.ProximityParams.ProximityHysteresis = RollerConstants.CAN_RANGE_HYSTERESIS;
    canRange.getConfigurator().apply(canConfig);

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
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrentVelocity(double velocity) {
    talon.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
  }

  @Override
  public boolean isCanRangeTriggered() {
    return canRange.getIsDetected().getValue();
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void runVelocity(double velocity) {
    talon.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
  }
}
