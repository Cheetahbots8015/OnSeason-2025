// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
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
  private final SoftwareLimitSwitchConfigs leaderSoftLimitConfig;
  private final SoftwareLimitSwitchConfigs followerSoftLimitConfig;
  private final TalonFXConfiguration leaderConfigs;
  private final TalonFXConfiguration followerConfigs;

  private final DigitalInput hallSensor =
      new DigitalInput(ElevatorConstants.ELEVATOR_HALL_SENSOR_ID);

  /* Gains */
  LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", ElevatorConstants.ELEVATOR_KA);
  LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", ElevatorConstants.ELEVATOR_KS);
  LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", ElevatorConstants.ELEVATOR_KV);
  LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ElevatorConstants.ELEVATOR_KP);
  LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ElevatorConstants.ELEVATOR_KI);
  LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ElevatorConstants.ELEVATOR_KD);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber(
          "Elevator/MotionAcceleration", ElevatorConstants.ELEVATOR_MOTION_MAGIC_ACCELERATION);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber(
          "Elevator/MotionCruiseVelocity", ElevatorConstants.ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY);
  LoggedTunableNumber motionJerk =
      new LoggedTunableNumber("Elevator/MotionJerk", ElevatorConstants.ELEVATOR_MOTION_MAGIC_JERK);

  LoggedTunableNumber manualVoltage = new LoggedTunableNumber("Elevator/manualVoltage", 0.0);
  LoggedTunableNumber positionTarget = new LoggedTunableNumber("Elevator/positionTarget", 0.0);

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

  private double[] encoderOffset = new double[] {0.0, 0.0};

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut =
      new VoltageOut(0.0)
          .withEnableFOC(true)
          .withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);

  private final MotionMagicVoltage motionMagicOut =
      new MotionMagicVoltage(0.0)
          .withEnableFOC(true)
          .withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);

  private final NeutralOut neutralOut = new NeutralOut();

  private final PositionVoltage positionVoltage =
      new PositionVoltage(0.0)
          .withEnableFOC(true)
          .withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);

  private final PositionTorqueCurrentFOC torqueCurrentPosition =
      new PositionTorqueCurrentFOC(0.0)
          .withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);

  private final DutyCycleOut DutyCycle =
      new DutyCycleOut(0.0)
          .withEnableFOC(true)
          .withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);

  public ElevatorSystemIOKrakenX60() {

    /* Instantiate motors and configurators */
    this.leader = new TalonFX(ElevatorConstants.ELEVATOR_LEFT_ID, "canivore");
    this.follower = new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_ID, "canivore");

    this.leaderConfigurator = leader.getConfigurator();
    this.followerConfigurator = follower.getConfigurator();

    /* Create configs */
    leaderSoftLimitConfig = new SoftwareLimitSwitchConfigs();
    leaderSoftLimitConfig.ForwardSoftLimitEnable =
        ElevatorConstants.ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_ENABLE;
    leaderSoftLimitConfig.ReverseSoftLimitEnable =
        ElevatorConstants.ELEVATOR_LEADER_REVERSE_SOFT_LIMIT_ENABLE;
    leaderSoftLimitConfig.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_SOFT_LIMIT_FORWARD;
    leaderSoftLimitConfig.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_SOFT_LIMIT_REVERSE;
    followerSoftLimitConfig = new SoftwareLimitSwitchConfigs();
    followerSoftLimitConfig.ForwardSoftLimitEnable =
        ElevatorConstants.ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_ENABLE;
    followerSoftLimitConfig.ReverseSoftLimitEnable =
        ElevatorConstants.ELEVATOR_FOLLOWER_REVERSE_SOFT_LIMIT_ENABLE;
    followerSoftLimitConfig.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_FOLLOWER_SOFT_LIMIT_FORWARD;
    followerSoftLimitConfig.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_FOLLOWER_SOFT_LIMIT_REVERSE;

    currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable =
        ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE;
    currentLimitsConfigs.SupplyCurrentLimitEnable =
        ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT_ENABLE;
    // if supply current limit is active for longer than this period, the supply
    // current will be adjusted to supply current lower limit
    currentLimitsConfigs.SupplyCurrentLowerTime =
        ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LOWER_TIME;

    leaderMotorConfigs = new MotorOutputConfigs();
    leaderMotorConfigs.Inverted =
        ElevatorConstants.ELEVATOR_LEFT_INVERSION
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leaderMotorConfigs.PeakReverseDutyCycle = ElevatorConstants.ELEVATOR_PEAK_REVERSE_DUTY_CYCLE;
    leaderMotorConfigs.NeutralMode =
        ElevatorConstants.ELEVATOR_LEFT_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    followerMotorConfigs = new MotorOutputConfigs();
    followerMotorConfigs.PeakForwardDutyCycle = ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE;
    followerMotorConfigs.NeutralMode =
        ElevatorConstants.ELEVATOR_RIGHT_BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;

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
    openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod =
        ElevatorConstants.ELEVATOR_DUTYCYCLE_OPENLOOP_RAMP_PERIOD;
    openLoopRampsConfigs.TorqueOpenLoopRampPeriod =
        ElevatorConstants.ELEVATOR_TORQUE_OPENLOOP_RAMP_PERIOD;
    openLoopRampsConfigs.VoltageOpenLoopRampPeriod =
        ElevatorConstants.ELEVATOR_VOLTAGE_OPENLOOP_RAMP_PERIOD;

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod =
        ElevatorConstants.ELEVATOR_DUTYCYCLE_CLOSEDLOOP_RAMP_PERIOD;
    closedLoopRampsConfigs.TorqueClosedLoopRampPeriod =
        ElevatorConstants.ELEVATOR_TORQUE_CLOSEDLOOP_RAMP_PERIOD;
    closedLoopRampsConfigs.VoltageClosedLoopRampPeriod =
        ElevatorConstants.ELEVATOR_VOLTAGE_CLOSEDLOOP_RAMP_PERIOD;

    leaderConfigs =
        new TalonFXConfiguration()
            .withOpenLoopRamps(openLoopRampsConfigs)
            .withClosedLoopRamps(closedLoopRampsConfigs)
            .withCurrentLimits(currentLimitsConfigs)
            .withMotorOutput(leaderMotorConfigs)
            .withSoftwareLimitSwitch(leaderSoftLimitConfig);
    followerConfigs =
        new TalonFXConfiguration()
            .withOpenLoopRamps(openLoopRampsConfigs)
            .withClosedLoopRamps(closedLoopRampsConfigs)
            .withCurrentLimits(currentLimitsConfigs)
            .withMotorOutput(followerMotorConfigs)
            .withSoftwareLimitSwitch(followerSoftLimitConfig);

    /* Apply configs */
    tryUntilOk(5, () -> leaderConfigurator.apply(leaderConfigs));
    tryUntilOk(5, () -> followerConfigurator.apply(followerConfigs));

    follower.setControl(
        new Follower(ElevatorConstants.ELEVATOR_LEFT_ID, ElevatorConstants.OPPOSE_MASTER)
            .withUpdateFreqHz(1000));

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
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
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
    tryUntilOk(
        5, () -> leader.optimizeBusUtilization(ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ, 0.1));
    tryUntilOk(
        5,
        () -> follower.optimizeBusUtilization(ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ, 0.1));
  }

  @Override
  public void updateInputs(ElevatorSystemIOInputs inputs) {
    SmartDashboard.putNumber(
        "position difference",
        leader.getPosition().getValueAsDouble() - follower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "torque difference",
        leader.getTorqueCurrent().getValueAsDouble()
            - follower.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "voltage difference",
        leader.getMotorVoltage().getValueAsDouble()
            - follower.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "duty difference",
        leader.getDutyCycle().getValueAsDouble() - follower.getDutyCycle().getValueAsDouble());
    inputs.connected =
        BaseStatusSignal.refreshAll(
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
    inputs.hallSensorActive = hallSensor.get();
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.accelerationRadsPerSec2 = Units.rotationsToRadians(acceleration.getValueAsDouble());
    inputs.appliedVoltage =
        new double[] {
          leftAppliedVoltage.getValueAsDouble(), rightAppliedVoltage.getValueAsDouble()
        };
    inputs.motionMagicPositionTarget =
        new double[] {leftReference.getValueAsDouble(), rightReference.getValueAsDouble()};
    inputs.supplyCurrentAmps =
        new double[] {leftSupplyCurrent.getValueAsDouble(), rightSupplyCurrent.getValueAsDouble()};
    inputs.torqueCurrentAmps =
        new double[] {leftTorqueCurrent.getValueAsDouble(), rightTorqueCurrent.getValueAsDouble()};
    inputs.tempCelcius =
        new double[] {leftTempCelsius.getValueAsDouble(), rightTempCelsius.getValueAsDouble()};
    inputs.appliedVoltage =
        new double[] {
          leftAppliedVoltage.getValueAsDouble(), rightAppliedVoltage.getValueAsDouble()
        };
    inputs.motionMagicPositionTarget =
        new double[] {leftReference.getValueAsDouble(), rightReference.getValueAsDouble()};
    inputs.supplyCurrentAmps =
        new double[] {leftSupplyCurrent.getValueAsDouble(), rightSupplyCurrent.getValueAsDouble()};
    inputs.torqueCurrentAmps =
        new double[] {leftTorqueCurrent.getValueAsDouble(), rightTorqueCurrent.getValueAsDouble()};
    inputs.tempCelcius =
        new double[] {leftTempCelsius.getValueAsDouble(), rightTempCelsius.getValueAsDouble()};
    inputs.encoderOffset = encoderOffset;
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
        || motionCruiseVelocity.hasChanged(0)
        || motionJerk.hasChanged(0)) {
      slot0Configs.kA = kA.get();
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
      motionMagicConfigs.MotionMagicJerk = motionJerk.get();

      tryUntilOk(5, () -> leaderConfigurator.apply(slot0Configs));
      tryUntilOk(5, () -> followerConfigurator.apply(slot0Configs));
      tryUntilOk(5, () -> leaderConfigurator.apply(motionMagicConfigs));
      tryUntilOk(5, () -> followerConfigurator.apply(motionMagicConfigs));
    }
  }

  @Override
  public void setVolts(double volts) {
    // leader.setControl(DutyCycle.withOutput(volts));
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setHeightRads(double height) {
    SmartDashboard.putNumber(
        "set height target", Units.radiansToRotations(height) + encoderOffset[0]);
    leader.setControl(
        motionMagicOut.withPosition(Units.radiansToRotations(height) + encoderOffset[0]));
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
  }

  @Override
  public void setEncoder2Zero() {
    SmartDashboard.putNumber("leaderposition", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("followerposition", follower.getPosition().getValueAsDouble());
    setEncoderOffset(
        leader.getPosition().getValueAsDouble(), leader.getPosition().getValueAsDouble());
  }

  @Override
  public boolean isHallSensorActive() {
    return !hallSensor.get();
  }

  @Override
  public void setSoftLimits(boolean enableForward, boolean enableReverse) {
    leaderSoftLimitConfig.ForwardSoftLimitEnable = enableForward;
    leaderSoftLimitConfig.ReverseSoftLimitEnable = enableReverse;
    leaderSoftLimitConfig.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_SOFT_LIMIT_FORWARD + encoderOffset[0];
    leaderSoftLimitConfig.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_SOFT_LIMIT_REVERSE + encoderOffset[0];
    tryUntilOk(5, () -> leaderConfigurator.apply(leaderSoftLimitConfig));
    followerSoftLimitConfig.ForwardSoftLimitEnable = enableForward;
    followerSoftLimitConfig.ReverseSoftLimitEnable = enableReverse;
    followerSoftLimitConfig.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_FOLLOWER_SOFT_LIMIT_FORWARD + encoderOffset[1];
    followerSoftLimitConfig.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_FOLLOWER_SOFT_LIMIT_REVERSE + encoderOffset[1];
    tryUntilOk(5, () -> followerConfigurator.apply(followerSoftLimitConfig));
  }

  @Override
  public void setEncoderOffset(double leaderOffset, double followerOffset) {
    encoderOffset = new double[] {leaderOffset, followerOffset};
  }

  @Override
  public void hold() {
    leader.setControl(motionMagicOut.withPosition(leader.getPosition().getValueAsDouble()));
  }
}
