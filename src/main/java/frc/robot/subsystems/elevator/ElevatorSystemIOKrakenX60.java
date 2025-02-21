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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.ElevatorConstants;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ElevatorSystemIOKrakenX60 implements ElevatorSystemIO {
  private final TalonFX leader;
  private final TalonFX follower;
  private final DigitalInput hallSensor =
      new DigitalInput(ElevatorConstants.ELEVATOR_HALL_SENSOR_ID);

  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<AngularAcceleration> leftAcceleration;
  private final StatusSignal<AngularAcceleration> rightAcceleration;
  private final StatusSignal<Voltage> leftAppliedVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Voltage> rightAppliedVoltage;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Double> leftReference;
  private final StatusSignal<Double> rightReference;
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
  private TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();

  private boolean homed = false;
  private double timer = -1.0;
  private double encoderOffset = 0.0;

  public ElevatorSystemIOKrakenX60() {
    /* Instantiate motors and configurators */
    this.leader =
        new TalonFX(ElevatorConstants.ELEVATOR_LEFT_ID, ElevatorConstants.ELEVATOR_CANNAME);
    this.follower =
        new TalonFX(ElevatorConstants.ELEVATOR_RIGHT_ID, ElevatorConstants.ELEVATOR_CANNAME);

    leaderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderConfigs.MotorOutput.withInverted(
        ElevatorConstants.ELEVATOR_INVERSION
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);
    followerConfigs.MotorOutput.withInverted(
        ElevatorConstants.ELEVATOR_INVERSION
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    followerConfigs.Slot1.kP = 2.0;
    leaderConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
    leaderConfigs.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.ELEVATOR_MOTION_MAGIC_ACCELERATION;
    leaderConfigs.MotorOutput.withPeakForwardDutyCycle(
        ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE);
    leaderConfigs.MotorOutput.withPeakReverseDutyCycle(
        ElevatorConstants.ELEVATOR_PEAK_REVERSE_DUTY_CYCLE);
    followerConfigs.MotorOutput.withPeakForwardDutyCycle(
        ElevatorConstants.ELEVATOR_PEAK_FOLLOWER_FORWARD_DUTY_CYCLE);
    followerConfigs.MotorOutput.withPeakReverseDutyCycle(
        ElevatorConstants.ELEVATOR_PEAK_FOLLOWER_REVERSE_DUTY_CYCLE);
    leaderConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.9;

    // follower differential control
    followerConfigs.DifferentialConstants.PeakDifferentialDutyCycle =
        ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE;
    followerConfigs.DifferentialConstants.PeakDifferentialVoltage = 4.0;
    followerConfigs.DifferentialSensors.DifferentialSensorSource =
        DifferentialSensorSourceValue.RemoteTalonFX_Diff;
    followerConfigs.DifferentialSensors.DifferentialTalonFXSensorID =
        ElevatorConstants.ELEVATOR_LEFT_ID;
    leader.getConfigurator().apply(leaderConfigs);
    follower.getConfigurator().apply(followerConfigs);

    leftPosition = leader.getPosition();
    leftVelocity = leader.getVelocity();
    leftAcceleration = leader.getAcceleration();
    leftAppliedVoltage = leader.getMotorVoltage();
    leftSupplyCurrent = leader.getSupplyCurrent();
    leftTorqueCurrent = leader.getTorqueCurrent();
    rightPosition = follower.getPosition();
    rightVelocity = follower.getVelocity();
    rightAcceleration = follower.getAcceleration();
    rightAppliedVoltage = follower.getMotorVoltage();
    rightSupplyCurrent = follower.getSupplyCurrent();
    rightTorqueCurrent = follower.getTorqueCurrent();
    leftReference = leader.getClosedLoopReference();
    rightReference = follower.getClosedLoopReference();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ,
                leftPosition,
                leftVelocity,
                leftAcceleration,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftTorqueCurrent,
                rightPosition,
                rightVelocity,
                rightAcceleration,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightTorqueCurrent,
                leftReference,
                rightReference));
    leader.optimizeBusUtilization(ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ, 0.1);
    follower.optimizeBusUtilization(ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ, 0.1);
  }

  @Override
  public void updateInputs(ElevatorSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAcceleration,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftTorqueCurrent,
                rightPosition,
                rightVelocity,
                rightAcceleration,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightTorqueCurrent,
                leftReference,
                rightReference)
            .isOK();

    inputs.positionDiff =
        leader.getPosition().getValueAsDouble() - follower.getPosition().getValueAsDouble();
    inputs.currentDiff =
        leader.getTorqueCurrent().getValueAsDouble()
            - follower.getTorqueCurrent().getValueAsDouble();
    inputs.voltageDiff =
        leader.getMotorVoltage().getValueAsDouble() - follower.getMotorVoltage().getValueAsDouble();
    inputs.dutyCycleDiff =
        leader.getDutyCycle().getValueAsDouble() - follower.getDutyCycle().getValueAsDouble();

    inputs.position =
        new double[] {leftPosition.getValueAsDouble(), rightPosition.getValueAsDouble()};
    inputs.velocity =
        new double[] {leftVelocity.getValueAsDouble(), rightVelocity.getValueAsDouble()};
    inputs.acceleration =
        new double[] {leftAcceleration.getValueAsDouble(), rightAcceleration.getValueAsDouble()};
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
    inputs.encoderOffset = encoderOffset;
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(voltageOut.withOutput(voltage));
    follower.setControl(new DifferentialVoltage(voltage, 0.0).withDifferentialSlot(1));
  }

  @Override
  public void setPosition(double position) {
    position += encoderOffset;
    if (Math.abs(leader.getPosition().getValueAsDouble() - position)
        < ElevatorConstants.ELEVATOR_POSITION_DEADBAND) {
      hold();
    } else if (Math.abs(leader.getPosition().getValueAsDouble() - position)
            < ElevatorConstants.ELEVATOR_CLOSE_POSITION_DEADBAND
        && leader.getPosition().getValueAsDouble() > position) {
      setVoltage(ElevatorConstants.ELEVATOR_LOW_DOWN_VOLTAGE);
    } else if (Math.abs(leader.getPosition().getValueAsDouble() - position)
            < ElevatorConstants.ELEVATOR_CLOSE_POSITION_DEADBAND
        && leader.getPosition().getValueAsDouble() <= position) {
      setVoltage(ElevatorConstants.ELEVATOR_LOW_UP_VOLTAGE);
    } else if (leader.getPosition().getValueAsDouble() > position) {
      setVoltage(ElevatorConstants.ELEVATOR_DOWN_VOLTAGE);
    } else {
      setVoltage(ElevatorConstants.ELEVATOR_UP_VOLTAGE);
    }
  }

  @Override
  public void hold() {
    leader.setControl(voltageOut.withOutput(ElevatorConstants.ELEVATOR_HOLD_VOLTAGE));
    follower.setControl(
        new DifferentialVoltage(ElevatorConstants.ELEVATOR_HOLD_VOLTAGE, 0.0)
            .withDifferentialSlot(1));
  }

  @Override
  public boolean isAtPosition(double pos) {
    pos += encoderOffset;
    return Math.abs(leader.getPosition().getValueAsDouble() - pos)
        < ElevatorConstants.ELEVATOR_POSITION_DEADBAND;
  }

  @Override
  public void setHome() {
    if (!homed) {
      if (timer == -1) {
        timer = Timer.getFPGATimestamp();
      }

      if (Timer.getFPGATimestamp() - timer < ElevatorConstants.HOME_UP_TIME) {
        setVoltage(ElevatorConstants.ELEVATOR_UP_VOLTAGE);
      } else {
        setVoltage(ElevatorConstants.ELEVATOR_DOWN_VOLTAGE);
        if (getHallSensorActive()) {
          resetOffset();
          stop();
          homed = true;
        }
      }
    }
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
    follower.setControl(neutralOut);
  }

  public boolean getHallSensorActive() {
    return !hallSensor.get();
  }

  @Override
  public boolean getHomed() {
    return homed;
  }

  private void resetOffset() {
    encoderOffset = leader.getPosition().getValueAsDouble();
  }
}
