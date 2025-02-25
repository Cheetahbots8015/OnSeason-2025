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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.MagicTimer;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ElevatorSystemIOKrakenX60 implements ElevatorSystemIO {
  private final TalonFX leader =
      new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID, ElevatorConstants.ELEVATOR_CANNAME);
  private final TalonFX follower =
      new TalonFX(ElevatorConstants.ELEVATOR_FOLLOWER_ID, ElevatorConstants.ELEVATOR_CANNAME);
  // hall sensor initialization
  private final DigitalInput hallSensor =
      new DigitalInput(ElevatorConstants.ELEVATOR_HALL_SENSOR_ID);

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<AngularAcceleration> leaderAcceleration;
  private final StatusSignal<AngularAcceleration> followerAcceleration;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Double> leaderReference;
  private final StatusSignal<Double> followerReference;

  private TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();

  // control methods initialization
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(50);
  private MotionMagicTorqueCurrentFOC motionMagic =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(200);
  private NeutralOut neutralOut = new NeutralOut();

  // used to zero the krankenX60s
  private double leaderEncoderOffset = 0.0;
  private double followerEncoderOffset = 0.0;
  private MagicTimer elevatorTimer = new MagicTimer();
  private MedianFilter filter = new MedianFilter(30);
  private homePhase systemHomePhase = homePhase.phase1;

  public ElevatorSystemIOKrakenX60() {
    leaderConfigs.MotorOutput.withNeutralMode(
        ElevatorConstants.ELEVATOR_NEUTRAL_MODE_COAST
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    followerConfigs.MotorOutput.withNeutralMode(
        ElevatorConstants.ELEVATOR_NEUTRAL_MODE_COAST
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);

    leaderConfigs.MotorOutput.withInverted(
        ElevatorConstants.ELEVATOR_INVERSION
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);
    followerConfigs.MotorOutput.withInverted(
        ElevatorConstants.ELEVATOR_INVERSION
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    leaderConfigs.Slot0.kP = ElevatorConstants.ELEVATOR_LEADER_KP;
    leaderConfigs.Slot0.kI = ElevatorConstants.ELEVATOR_LEADER_KI;
    leaderConfigs.Slot0.kD = ElevatorConstants.ELEVATOR_LEADER_KD;
    leaderConfigs.Slot0.kS = ElevatorConstants.ELEVATOR_LEADER_KS;
    leaderConfigs.Slot0.kA = ElevatorConstants.ELEVATOR_LEADER_KA;
    leaderConfigs.Slot0.kV = ElevatorConstants.ELEVATOR_LEADER_KV;
    followerConfigs.Slot0.kP = ElevatorConstants.ELEVATOR_FOLLOWER_KP;
    followerConfigs.Slot0.kI = ElevatorConstants.ELEVATOR_FOLLOWER_KI;
    followerConfigs.Slot0.kD = ElevatorConstants.ELEVATOR_FOLLOWER_KD;
    followerConfigs.Slot0.kS = ElevatorConstants.ELEVATOR_FOLLOWER_KS;
    followerConfigs.Slot0.kA = ElevatorConstants.ELEVATOR_FOLLOWER_KA;
    followerConfigs.Slot0.kV = ElevatorConstants.ELEVATOR_FOLLOWER_KV;

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderAcceleration = leader.getAcceleration();
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    leaderReference = leader.getClosedLoopReference();
    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerAcceleration = follower.getAcceleration();
    followerAppliedVoltage = follower.getMotorVoltage();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerReference = follower.getClosedLoopReference();

    // config motionmagic
    leaderConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.ELEVATOR_LEADER_CRUISE_VELOCITY;
    leaderConfigs.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.ELEVATOR_LEADER_CRUISE_ACCELERATION;
    followerConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.ELEVATOR_FOLLOWER_CRUISE_VELOCITY;
    followerConfigs.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.ELEVATOR_FOLLOWER_CRUISE_ACCELERATION;

    // config ducy cycle limit
    leaderConfigs.MotorOutput.withPeakForwardDutyCycle(
        ElevatorConstants.ELEVATOR_FORWARD_DUTY_CYCLE_LIMIT);
    leaderConfigs.MotorOutput.withPeakReverseDutyCycle(
        ElevatorConstants.ELEVATOR_REVERSE_DUTY_CYCLE_LIMIT);
    followerConfigs.MotorOutput.withPeakForwardDutyCycle(
        ElevatorConstants.ELEVATOR_FORWARD_DUTY_CYCLE_LIMIT);
    followerConfigs.MotorOutput.withPeakReverseDutyCycle(
        ElevatorConstants.ELEVATOR_REVERSE_DUTY_CYCLE_LIMIT);

    // config softlimit
    // hall sensor is used as the reverse limit
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        ElevatorConstants.ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_ENABLE;
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_THRESHOLD
            + leader.getPosition().getValueAsDouble();
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        ElevatorConstants.ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_ENABLE;
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_THRESHOLD
            + follower.getPosition().getValueAsDouble();

    // follower differential control
    followerConfigs.Slot1.kP = ElevatorConstants.ELEVATOR_FOLLOWER_DIFFERENTIAL_KP;
    followerConfigs.DifferentialConstants.PeakDifferentialDutyCycle =
        ElevatorConstants.ELEVATOR_FORWARD_DUTY_CYCLE_LIMIT;
    followerConfigs.DifferentialConstants.PeakDifferentialVoltage =
        ElevatorConstants.ELEVATOR_PEAK_DIFFERENTIAL_VOLTAGE;
    followerConfigs.DifferentialSensors.DifferentialSensorSource =
        DifferentialSensorSourceValue.RemoteTalonFX_Diff;
    followerConfigs.DifferentialSensors.DifferentialTalonFXSensorID =
        ElevatorConstants.ELEVATOR_LEADER_ID;

    // apply configs
    tryUntilOk(5, () -> leader.getConfigurator().apply(leaderConfigs));
    tryUntilOk(5, () -> follower.getConfigurator().apply(followerConfigs));

    // initialize offset
    leaderEncoderOffset = leader.getPosition().getValueAsDouble();
    followerEncoderOffset = follower.getPosition().getValueAsDouble();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                ElevatorConstants.ELEVATOR_SIGNAL_UPDATE_FREQUENCY_HZ,
                leaderPosition,
                leaderVelocity,
                leaderAcceleration,
                leaderAppliedVoltage,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                leaderReference,
                followerPosition,
                followerVelocity,
                followerAcceleration,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerReference));
    leader.optimizeBusUtilization(ElevatorConstants.ELEVATOR_SIGNAL_UPDATE_FREQUENCY_HZ, 0.1);
    follower.optimizeBusUtilization(ElevatorConstants.ELEVATOR_SIGNAL_UPDATE_FREQUENCY_HZ, 0.1);
  }

  @Override
  public void updateInputs(ElevatorSystemIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAcceleration,
                leaderAppliedVoltage,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                leaderReference,
                followerPosition,
                followerVelocity,
                followerAcceleration,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerReference)
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
        new double[] {leaderPosition.getValueAsDouble(), followerPosition.getValueAsDouble()};
    inputs.positionWithoutOffset =
        new double[] {
          leaderPosition.getValueAsDouble() - leaderEncoderOffset,
          followerPosition.getValueAsDouble() - followerEncoderOffset
        };
    inputs.velocity =
        new double[] {leaderVelocity.getValueAsDouble(), followerVelocity.getValueAsDouble()};
    inputs.acceleration =
        new double[] {
          leaderAcceleration.getValueAsDouble(), followerAcceleration.getValueAsDouble()
        };
    inputs.appliedVoltage =
        new double[] {
          leaderAppliedVoltage.getValueAsDouble(), followerAppliedVoltage.getValueAsDouble()
        };
    inputs.motionMagicPositionTarget =
        new double[] {leaderReference.getValueAsDouble(), followerReference.getValueAsDouble()};
    inputs.supplyCurrentAmps =
        new double[] {
          leaderSupplyCurrent.getValueAsDouble(), followerSupplyCurrent.getValueAsDouble()
        };
    inputs.torqueCurrentAmps =
        new double[] {
          leaderTorqueCurrent.getValueAsDouble(), followerTorqueCurrent.getValueAsDouble()
        };
    inputs.encoderOffset = new double[] {leaderEncoderOffset, followerEncoderOffset};
  }

  private double getLeaderPositionWithoutOffset() {
    return leader.getPosition().getValueAsDouble() - leaderEncoderOffset;
  }

  private double getFollowerPositionWithoutOffset() {
    return follower.getPosition().getValueAsDouble() - followerEncoderOffset;
  }

  private void updateOffset() {
    leaderEncoderOffset = leader.getPosition().getValueAsDouble();
    followerEncoderOffset = follower.getPosition().getValueAsDouble();
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_THRESHOLD
            + leader.getPosition().getValueAsDouble();
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_THRESHOLD
            + follower.getPosition().getValueAsDouble();
    leader.getConfigurator().apply(leaderConfigs);
    follower.getConfigurator().apply(followerConfigs);
  }

  private boolean getHallSensorActive() {
    return !hallSensor.get();
  }

  public void resetFilter() {
    filter.reset();
  }

  public void resetHomePhase() {
    systemHomePhase = homePhase.phase1;
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
    follower.setControl(neutralOut);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(voltageOut.withOutput(voltage).withLimitReverseMotion(getHallSensorActive()));
    follower.setControl(
        new DifferentialVoltage(voltage, 0.0)
            .withDifferentialSlot(1)
            .withUpdateFreqHz(50.0)
            .withLimitReverseMotion(getHallSensorActive()));
  }

  @Override
  public void setPosition_MotionMagic(double position) {
    leader.setControl(
        motionMagic
            .withPosition(position + leaderEncoderOffset)
            .withLimitReverseMotion(getHallSensorActive())
            .withLimitForwardMotion(
                this.getLeaderPositionWithoutOffset()
                    > ElevatorConstants.ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_THRESHOLD));
    follower.setControl(
        motionMagic
            .withPosition(position + followerEncoderOffset)
            .withLimitReverseMotion(getHallSensorActive())
            .withLimitForwardMotion(
                this.getFollowerPositionWithoutOffset()
                    > ElevatorConstants.ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_THRESHOLD));
  }

  @Override
  public void setPosition_MotionMagician(double position) {
    if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
        < ElevatorConstants.ELEVATOR_POSITION_DEADBAND) {
      setVoltage(ElevatorConstants.ELEVATOR_HOLD_VOLTAGE);
    } else if (this.getLeaderPositionWithoutOffset()
        > position + ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_CLOSE_POSITION_DEADBAND_L3L4) {
      setVoltage(
          (filter.calculate(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_HIGH_DOWN_VOLTAGE_L3L4)));
    } else if (this.getLeaderPositionWithoutOffset()
        < position - ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_CLOSE_POSITION_DEADBAND_L3L4) {
      setVoltage(filter.calculate(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_HIGH_UP_VOLTAGE_L3L4));
    } else if (this.getLeaderPositionWithoutOffset() > position) {
      setVoltage(
          filter.calculate(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_LOW_DOWN_VOLTAGE_L3L4));
    } else {
      setVoltage(filter.calculate(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_LOW_UP_VOLTAGE_L3L4));
    }
  }

  @Override
  // no need to use median filter for short distance movement
  public void setPosition_MotionMagicianLow(double position) {
    if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
        < ElevatorConstants.ELEVATOR_POSITION_DEADBAND) {
      setVoltage(ElevatorConstants.ELEVATOR_HOLD_VOLTAGE);
    } else if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
            < ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_CLOSE_POSITION_DEADBAND_L1L2
        && this.getLeaderPositionWithoutOffset() > position) {
      setVoltage(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_LOW_DOWN_VOLTAGE_L1L2);
    } else if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
            < ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_CLOSE_POSITION_DEADBAND_L1L2
        && this.getLeaderPositionWithoutOffset() <= position) {
      setVoltage(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_LOW_UP_VOLTAGE_L1L2);
    } else if (this.getLeaderPositionWithoutOffset() > position) {
      setVoltage(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_HIGH_DOWN_VOLTAGE_L1L2);
    } else {
      setVoltage(ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_HIGH_UP_VOLTAGE_L1L2);
    }
  }

  @Override
  public boolean isAtPosition(double height) {
    return Math.abs(this.getFollowerPositionWithoutOffset() - height)
        < ElevatorConstants.ELEVATOR_POSITION_DEADBAND;
  }

  public boolean isAbovePosition(double height) {
    return this.getLeaderPositionWithoutOffset() > height;
  }

  @Override
  public void setHome() {
    if (systemHomePhase == homePhase.phase1) {
      if (getHallSensorActive()) {
        elevatorTimer.resetTimer();
        elevatorTimer.startTimer();
        systemHomePhase = homePhase.phase2;
      }
    } else if (systemHomePhase == homePhase.phase2) {
      if (elevatorTimer.getTimePassedSec() < ElevatorConstants.ELEVATOR_HOME_UP_TIME) {
        setVoltage(ElevatorConstants.ELEVATOR_HOME_UP_VOLTAGE);
      } else {
        if (getHallSensorActive()) {
          updateOffset();
          resetHomePhase();
          stop();
        } else {
          setVoltage(ElevatorConstants.ELEVATOR_HOME_DOWN_VOLTAGE);
        }
      }
    } else if (systemHomePhase == homePhase.phase3) {
      if (getHallSensorActive()) {
        updateOffset();
        resetHomePhase();
        stop();
      } else {
        setVoltage(ElevatorConstants.ELEVATOR_HOME_DOWN_VOLTAGE);
      }
    }
  }

  @Override
  public void defaultFall() {
    if (this.getLeaderPositionWithoutOffset()
        > ElevatorConstants.ELEVATOR_DEFAULT_SHUT_DOWN_POSITION) {
      setVoltage(ElevatorConstants.ELEVATOR_DEFAULT_FALL_VOLTAGE);
    } else {
      stop();
    }
  }

  private enum homePhase {
    phase1,
    phase2,
    phase3
  }
}
