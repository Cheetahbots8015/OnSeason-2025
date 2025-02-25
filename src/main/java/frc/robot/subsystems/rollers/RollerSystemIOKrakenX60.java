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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RollerConstants;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class RollerSystemIOKrakenX60 implements RollerSystemIO {
  private final TalonFX roller =
      new TalonFX(RollerConstants.ROLLER_ID, RollerConstants.ROLLER_CANNAME);
  private final CANrange canRange =
      new CANrange(RollerConstants.ROLLER_CANRANGE_ID, RollerConstants.ROLLER_CANNAME);

  private final TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
  private final CANrangeConfiguration canConfig = new CANrangeConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  public RollerSystemIOKrakenX60() {

    // config neutralmode
    rollerConfigs.MotorOutput.withNeutralMode(
        RollerConstants.ROLLER_NEUTRAL_MODE_COAST
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    // config direction
    // POSITIVE for intaking and placing the corals
    rollerConfigs.MotorOutput.withInverted(
        RollerConstants.ROLLER_INVERSION
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    // config PIDSAV
    rollerConfigs.Slot0.kP = RollerConstants.ROLLER_KP;
    rollerConfigs.Slot0.kI = RollerConstants.ROLLER_KI;
    rollerConfigs.Slot0.kD = RollerConstants.ROLLER_KD;
    rollerConfigs.Slot0.kA = RollerConstants.ROLLER_KA;
    rollerConfigs.Slot0.kS = RollerConstants.ROLLER_KS;
    rollerConfigs.Slot0.kV = RollerConstants.ROLLER_KV;

    // apply roller configs
    roller.getConfigurator().apply(rollerConfigs);

    // config canRange
    canConfig.ProximityParams.ProximityThreshold = RollerConstants.CANRANGE_DISTANCE_THRESHOLD;
    canConfig.ProximityParams.MinSignalStrengthForValidMeasurement =
        RollerConstants.CANRANGE_MIN_SIGNAL_STRENGTH;
    canConfig.ProximityParams.ProximityHysteresis = RollerConstants.CANRANGE_HYSTERESIS;

    // apply canRange configs
    canRange.getConfigurator().apply(canConfig);

    position = roller.getPosition();
    velocity = roller.getVelocity();
    acceleration = roller.getAcceleration();
    appliedVoltage = roller.getMotorVoltage();
    supplyCurrent = roller.getSupplyCurrent();
    torqueCurrent = roller.getTorqueCurrent();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                RollerConstants.ROLLER_SIGNAL_UPDATE_FREQUENCY_HZ,
                position,
                velocity,
                acceleration,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent));
    tryUntilOk(5, () -> roller.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, acceleration, appliedVoltage, supplyCurrent, torqueCurrent)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.accelerationRadsPerSec2 = Units.rotationsToRadians(acceleration.getValueAsDouble());
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    roller.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrentVelocity(double velocity) {
    roller.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
  }

  @Override
  public boolean isCanRangeTriggered() {
    return canRange.getIsDetected().getValue();
  }

  @Override
  public void stop() {
    roller.setControl(neutralOut);
  }

  @Override
  public void runVelocity(double velocity) {
    roller.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
  }
}
