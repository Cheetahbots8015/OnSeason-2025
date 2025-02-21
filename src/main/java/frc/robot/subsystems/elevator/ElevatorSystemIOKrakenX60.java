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

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ElevatorConstants;

/**
 * Generic roller IO implementation for a roller or series of rollers using a Kraken.
 */

/**
 * Generic roller IO implementation for a roller or series of rollers using a
 * Kraken.
 */
public class ElevatorSystemIOKrakenX60 implements ElevatorSystemIO {
	private final TalonFX leader;
	private final TalonFX follower;
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
	// Single shot for voltage mode, robot loop will call continuously
	private final VoltageOut voltageOut =
			new VoltageOut(0.0)

					.withEnableFOC(true)
					.withUpdateFreqHz(ElevatorConstants.CONTROL_UPDATE_FREQUENCY_HZ);
	private final NeutralOut neutralOut = new NeutralOut();
	private TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
	private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();

  private boolean homed = false;
  private double timer = -1.0;
  private double encoderOffset = 0.0;
  private double lastPosition = -100.0; // -100.0 for empty value to start
  private double motionMagicianVoltage = 0.0;

  MedianFilter filter;

	public ElevatorSystemIOKrakenX60() {
		filter = new MedianFilter(5);/* Instantiate motors and configurators */
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

		followerConfigs.Slot1.kP = ElevatorConstants.ELEVATOR_FOLLOWER_KP;

		leaderConfigs.MotorOutput.withPeakForwardDutyCycle(
				ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE);
		leaderConfigs.MotorOutput.withPeakReverseDutyCycle(
				ElevatorConstants.ELEVATOR_PEAK_REVERSE_DUTY_CYCLE);
		followerConfigs.MotorOutput.withPeakForwardDutyCycle(
				ElevatorConstants.ELEVATOR_PEAK_FOLLOWER_FORWARD_DUTY_CYCLE);
		followerConfigs.MotorOutput.withPeakReverseDutyCycle(
				ElevatorConstants.ELEVATOR_PEAK_FOLLOWER_REVERSE_DUTY_CYCLE);
		leaderConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = ElevatorConstants.ELEVATOR_OPEN_LOOP_RAMP_PERIOD;

		// follower differential control
		followerConfigs.DifferentialConstants.PeakDifferentialDutyCycle =
				ElevatorConstants.ELEVATOR_PEAK_FORWARD_DUTY_CYCLE;
		followerConfigs.DifferentialConstants.PeakDifferentialVoltage = ElevatorConstants.ELEVATOR_PEAK_DIFFERENTIAL_VOLTAGE;
		followerConfigs.DifferentialSensors.DifferentialSensorSource =
				DifferentialSensorSourceValue.RemoteTalonFX_Diff;
		followerConfigs.DifferentialSensors.DifferentialTalonFXSensorID =
				ElevatorConstants.ELEVATOR_LEFT_ID;
		leader.getConfigurator().apply(leaderConfigs);
		follower.getConfigurator().apply(followerConfigs);

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderAcceleration = leader.getAcceleration();
    leaderAppliedVoltage = leader.getMotorVoltage();
    leaderSupplyCurrent = leader.getSupplyCurrent();
    leaderTorqueCurrent = leader.getTorqueCurrent();
    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerAcceleration = follower.getAcceleration();
    followerAppliedVoltage = follower.getMotorVoltage();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTorqueCurrent = follower.getTorqueCurrent();
    leaderReference = leader.getClosedLoopReference();
    followerReference = follower.getClosedLoopReference();

    tryUntilOk(
        5,
        () -> BaseStatusSignal.setUpdateFrequencyForAll(
            ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ,
            leaderPosition,
            leaderVelocity,
            leaderAcceleration,
            leaderAppliedVoltage,
            leaderSupplyCurrent,
            leaderTorqueCurrent,
            followerPosition,
            followerVelocity,
            followerAcceleration,
            followerAppliedVoltage,
            followerSupplyCurrent,
            followerTorqueCurrent,
            leaderReference,
            followerReference));
    leader.optimizeBusUtilization(ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ, 0.1);
    follower.optimizeBusUtilization(ElevatorConstants.SIGNAL_UPDATE_FREQUENCY_HZ, 0.1);
  }

  @Override
  public void updateInputs(ElevatorSystemIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(
        leaderPosition,
        leaderVelocity,
        leaderAcceleration,
        leaderAppliedVoltage,
        leaderSupplyCurrent,
        leaderTorqueCurrent,
        followerPosition,
        followerVelocity,
        followerAcceleration,
        followerAppliedVoltage,
        followerSupplyCurrent,
        followerTorqueCurrent,
        leaderReference,
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

    inputs.position = new double[] { leaderPosition.getValueAsDouble(), followerPosition.getValueAsDouble() };
    inputs.velocity = new double[] { leaderVelocity.getValueAsDouble(), followerVelocity.getValueAsDouble() };
    inputs.acceleration = new double[] { leaderAcceleration.getValueAsDouble(),
        followerAcceleration.getValueAsDouble() };
    inputs.appliedVoltage = new double[] {
        leaderAppliedVoltage.getValueAsDouble(), followerAppliedVoltage.getValueAsDouble()
    };
    inputs.motionMagicPositionTarget = new double[] { leaderReference.getValueAsDouble(),
        followerReference.getValueAsDouble() };
    inputs.supplyCurrentAmps = new double[] { leaderSupplyCurrent.getValueAsDouble(),
        followerSupplyCurrent.getValueAsDouble() };
    inputs.torqueCurrentAmps = new double[] { leaderTorqueCurrent.getValueAsDouble(),
        followerTorqueCurrent.getValueAsDouble() };
    inputs.encoderOffset = encoderOffset;
  }

	@Override
	public void setVoltage(double voltage) {
		leader.setControl(voltageOut.withOutput(voltage));
		follower.setControl(new DifferentialVoltage(voltage, 0.0).withDifferentialSlot(1));
	}

  @Override
  public void setPosition(double position) {
    if ((lastPosition < (position + encoderOffset)
        && leader.getPosition().getValueAsDouble() > (position + encoderOffset))
        || (lastPosition > (position + encoderOffset)
            && leader.getPosition().getValueAsDouble() < (position + encoderOffset))) {
      lastPosition = -100.0;
    }
    if (lastPosition == -100.0) {
      lastPosition = leader.getPosition().getValueAsDouble();
      motionMagicianVoltage = 0.0;
    }
    if (lastPosition < (position + encoderOffset)) {
      if ((leader.getPosition().getValueAsDouble() - lastPosition) < (position + encoderOffset - lastPosition) / 2) {
        motionMagicianVoltage += ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_UP_RATE;
      } else {
        motionMagicianVoltage -= ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_UP_RATE;
      }
      if(motionMagicianVoltage > ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_UP_MAXIMUM) {
        motionMagicianVoltage = ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_UP_MAXIMUM;
      }
    }else{
      if ((lastPosition - leader.getPosition().getValueAsDouble()) < (lastPosition - position - encoderOffset) / 2) {
        motionMagicianVoltage += ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_DOWN_RATE;
      } else {
        motionMagicianVoltage -= ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_DOWN_RATE;
      }
      if(motionMagicianVoltage < ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_DOWN_MAXIMUM) {
        motionMagicianVoltage = ElevatorConstants.ELEVATOR_MOTION_MAGICIAN_DOWN_MAXIMUM;
      }
    }
    setVoltage(filter.calculate(motionMagicianVoltage));
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
    return Math.abs(leader.getPosition().getValueAsDouble() - pos) < ElevatorConstants.ELEVATOR_POSITION_DEADBAND;
  }

	@Override
	public void setHome() {
		if (!homed) {
			if (timer == -1) {
				timer = Timer.getFPGATimestamp();
			}

			if (Timer.getFPGATimestamp() - timer < ElevatorConstants.ELEVATOR_HOME_UP_TIME) {
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
