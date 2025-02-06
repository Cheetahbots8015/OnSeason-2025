// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.lang.ref.Reference;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.ElevatorConstants;
import frc.robot.subsystems.rollers.RollerSystemIO.RollerSystemIOInputs;
import frc.robot.util.LoggedTunableNumber;


/**
 * Generic roller IO implementation for a roller or series of rollers using a
 * Kraken.
 */
public class ElevatorSystemIOKrakenX60 implements ElevatorSystemIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private TalonFXConfigurator leaderConfigurator;
  private TalonFXConfigurator followerConfigurator;

  private final CurrentLimitsConfigs currentLimitsConfigs;
  private final MotorOutputConfigs leaderMotorConfigs;
  private final MotorOutputConfigs followerMotorConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs motionMagicConfigs;

  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", ElevatorConstants.ELEVATOR_KA);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", ElevatorConstants.ELEVATOR_KS);
  LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", ElevatorConstants.ELEVATOR_KV);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ElevatorConstants.ELEVATOR_KP);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ElevatorConstants.ELEVATOR_KI);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ElevatorConstants.ELEVATOR_KD);

  LoggedTunableNumber motionAcceleration = new LoggedTunableNumber("Elevator/MotionAcceleration",
      ElevatorConstants.ELEVATOR_MOTION_MAGIC_ACCELERATION);
  LoggedTunableNumber motionCruiseVelocity = new LoggedTunableNumber("Elevator/MotionCruiseVelocity",
      ElevatorConstants.ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY);
  LoggedTunableNumber motionJerk = new LoggedTunableNumber("Elevator/MotionJerk",
      ElevatorConstants.ELEVATOR_MOTION_MAGIC_JERK);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> leftAppliedVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Voltage> rightAppliedVoltage;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Temperature> leftTempCelsius;
  private final StatusSignal<Temperature> rightTempCelsius;
  private final StatusSignal<Double> leftReference;
  private final StatusSignal<Double> rightReference;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);
  private final MotionMagicTorqueCurrentFOC motionMagicOut = new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);
  private final NeutralOut neutralOut = new NeutralOut();

  public ElevatorSystemIOKrakenX60() {

    /* Instantiate motors and configurators */
    this.leader = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_ID, "dabus");
    this.follower = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_ID, "dabus");

    this.leaderConfigurator = leader.getConfigurator();
    this.followerConfigurator = follower.getConfigurator();

    /* Create configs */
    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE;
    currentLimitsConfigs.SupplyCurrentLimitEnable = ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT_ENABLE;
    currentLimitsConfigs.StatorCurrentLimit = ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT_AMPS;
    currentLimitsConfigs.SupplyCurrentLimit = ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    currentLimitsConfigs.SupplyCurrentLowerLimit = ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    // if supply current limit is active for longer than this period, the supply
    // current will be adjusted to supply current lower limit
    currentLimitsConfigs.SupplyCurrentLowerTime = ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LOWER_TIME;

    leaderMotorConfigs = new MotorOutputConfigs();
    leaderMotorConfigs.Inverted = ElevatorConstants.ELEVATOR_LEFT_INVERSION ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    leaderMotorConfigs.PeakForwardDutyCycle = ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE;
    leaderMotorConfigs.PeakReverseDutyCycle = ElevatorConstants.ELEVATOR_PEAK_REVERSE_DUTY_CYCLE;
    leaderMotorConfigs.NeutralMode = ElevatorConstants.ELEVATOR_LEFT_BRAKE ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;

    followerMotorConfigs = new MotorOutputConfigs();
    followerMotorConfigs.PeakForwardDutyCycle = ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE;
    followerMotorConfigs.PeakReverseDutyCycle = ElevatorConstants.ELEVATOR_PEAK_REVERSE_DUTY_CYCLE;
    followerMotorConfigs.NeutralMode = ElevatorConstants.ELEVATOR_RIGHT_BRAKE ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;

    slot0Configs = new Slot0Configs();
    slot0Configs.kA = kA.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();

    motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = motionJerk.get();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = ElevatorConstants.ELEVATOR_DUTYCYCLE_CLOSEDLOOP_RAMP_PERIOD;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod = ElevatorConstants.ELEVATOR_TORQUE_CLOSEDLOOP_RAMP_PERIOD;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod = ElevatorConstants.ELEVATOR_VOLTAGE_CLOSEDLOOP_RAMP_PERIOD;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = ElevatorConstants.ELEVATOR_DUTYCYCLE_CLOSEDLOOP_RAMP_PERIOD;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = ElevatorConstants.ELEVATOR_TORQUE_CLOSEDLOOP_RAMP_PERIOD;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = ElevatorConstants.ELEVATOR_VOLTAGE_CLOSEDLOOP_RAMP_PERIOD;

    /* Apply configs */
    tryUntilOk(5, () -> leaderConfigurator.apply(currentLimitsConfigs));
    tryUntilOk(5, () -> leaderConfigurator.apply(leaderMotorConfigs));
    tryUntilOk(5, () -> leaderConfigurator.apply(slot0Configs));
    tryUntilOk(5, () -> leaderConfigurator.apply(motionMagicConfigs));
    tryUntilOk(5, () -> leaderConfigurator.apply(openLoopRampsConfigs));
    tryUntilOk(5, () -> leaderConfigurator.apply(closedLoopRampsConfigs));

    tryUntilOk(5, () -> followerConfigurator.apply(currentLimitsConfigs));
    tryUntilOk(5, () -> followerConfigurator.apply(followerMotorConfigs));
    tryUntilOk(5, () -> followerConfigurator.apply(slot0Configs));
    tryUntilOk(5, () -> followerConfigurator.apply(motionMagicConfigs));
    tryUntilOk(5, () -> followerConfigurator.apply(openLoopRampsConfigs));
    tryUntilOk(5, () -> followerConfigurator.apply(closedLoopRampsConfigs));

    follower.setControl(new Follower(ElevatorConstants.ELEVATOR_LEFT_ID, ElevatorConstants.OPPOSE_MASTER));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    acceleration = leader.getAcceleration();
    leftAppliedVoltage = leader.getMotorVoltage();
    leftSupplyCurrent = leader.getSupplyCurrent();
    leftTorqueCurrent = leader.getTorqueCurrent();
    rightAppliedVoltage = follower.getMotorVoltage();
    rightSupplyCurrent = follower.getSupplyCurrent();
    rightTorqueCurrent = follower.getTorqueCurrent();
    leftTempCelsius = leader.getDeviceTemp();
    rightTempCelsius = follower.getDeviceTemp();
    leftReference = leader.getClosedLoopReference();
    rightReference = follower.getClosedLoopReference();

    tryUntilOk(
        5,
        () -> BaseStatusSignal.setUpdateFrequencyForAll(
            ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ,
            position,
            velocity,
            acceleration,
            leftAppliedVoltage,
            leftSupplyCurrent,
            leftTorqueCurrent,
            rightAppliedVoltage,
            rightSupplyCurrent,
            rightTorqueCurrent,
            leftTempCelsius,
            rightTempCelsius,
            leftReference,
            rightReference));
    tryUntilOk(5, () -> leader.optimizeBusUtilization(0, 1.0));
    tryUntilOk(5, () -> follower.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(ElevatorSystemIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(
        position,
        velocity,
        acceleration,
        leftAppliedVoltage,
        leftSupplyCurrent,
        leftTorqueCurrent,
        rightAppliedVoltage,
        rightSupplyCurrent,
        rightTorqueCurrent,
        leftTempCelsius,
        rightTempCelsius,
        leftReference,
        rightReference)
        .isOK();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.accelerationRadsPerSec2 = Units.rotationsToRadians(acceleration.getValueAsDouble());
    inputs.appliedVoltage = new double[] { leftAppliedVoltage.getValueAsDouble(), rightAppliedVoltage.getValueAsDouble() };
    inputs.motionMagicPositionTarget = new double[] { leftReference.getValueAsDouble(), rightReference.getValueAsDouble() };
    inputs.supplyCurrentAmps = new double[] { leftSupplyCurrent.getValueAsDouble(), rightSupplyCurrent.getValueAsDouble() };
    inputs.torqueCurrentAmps = new double[] { leftTorqueCurrent.getValueAsDouble(), rightTorqueCurrent.getValueAsDouble() };
    inputs.tempCelcius = new double[] { leftTempCelsius.getValueAsDouble(), rightTempCelsius.getValueAsDouble() };
  }

  @Override
  public void updateTunableNumbers() {
    if (kA.hasChanged(0)
        || kS.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)
        || motionAcceleration.hasChanged(0)
        || motionCruiseVelocity.hasChanged(0)) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      tryUntilOk(5, () -> leaderConfigurator.apply(slot0Configs));
      tryUntilOk(5, () -> followerConfigurator.apply(slot0Configs));
      tryUntilOk(5, () -> leaderConfigurator.apply(motionMagicConfigs));
      tryUntilOk(5, () -> followerConfigurator.apply(motionMagicConfigs));
    }
  }

  @Override
  public void setVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setHeightRads(double height) {
    leader.setControl(motionMagicOut.withPosition(Units.radiansToRotations(height)));
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
  }

  @Override
  public void setEncoder2Zero(){
    leader.setPosition(0.0);
    follower.setPosition(0.0);
  }

  @Override
  public double getEncoderPositionRads(){
    return Units.rotationsToRadians(position.getValueAsDouble());
  }
}
