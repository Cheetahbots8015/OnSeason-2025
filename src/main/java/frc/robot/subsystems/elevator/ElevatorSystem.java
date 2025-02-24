// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorSystem extends SubsystemBase {
	protected final ElevatorSystemIOInputsAutoLogged inputs = new ElevatorSystemIOInputsAutoLogged();
	private final String name;
	private final ElevatorSystemIO io;
	private final Alert disconnected;

	private ElevatorState systemState = ElevatorState.IDLE;
	private ElevatorState nextSystemState = ElevatorState.IDLE;
	private ElevatorPosition systemPosition = ElevatorPosition.NULL;

	public ElevatorSystem(String name, ElevatorSystemIO io) {
		this.name = name;
		this.io = io;
		disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			nextSystemState = ElevatorState.IDLE;
		}

		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);
		Logger.recordOutput(name, this.getElevatorState());
		disconnected.set(!inputs.connected);

		updateStateMachine();
	}

	public void updateStateMachine() {
		if (systemState != nextSystemState) {
			systemState = nextSystemState;
			// TODO: change of state sanity check
		}

		switch (systemState) {
			case IDLE:
				io.defaultFall();
				break;
			case HOLD:
				io.hold();
				break;
			case HOMING:
				io.setHome();
				break;
			case MANUALUPWARD:
				io.setVoltage(ElevatorConstants.ELEVATOR_UP_VOLTAGE);
				break;
			case MANUTALDOWNWARD:
				io.setVoltage(ElevatorConstants.ELEVATOR_DOWN_VOLTAGE);
				break;
			case POSITION:
				switch (systemPosition) {
					case L1:
						io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L1_POSITION);
						break;
					case L2:
						io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L2_POSITION);
						break;
					case L3:
						io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L3_POSITION);
						break;
					case L4:
						io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L4_POSITION);
						break; // TODO:need to add further out
					case LOW_ALGAE:
						io.setPosition_MotionMagician(ElevatorConstants.ELEVATOR_LOW_ALGAE_POSITION);
						break;
					case HIGH_ALGAE:
						io.setPosition_MotionMagician(ElevatorConstants.ELEVATOR_HIGH_ALGAE_POSITION);
						break;
					default:
						io.defaultFall();
						break;
				}
				break;
		}
	}

	/* Returns the current system state of the elevator */
	public ElevatorState getElevatorState() {
		return systemState;
	}

	public void setElevatorState(ElevatorState state) {
		nextSystemState = state;
	}

	public void setElevatorPosition(ElevatorPosition pos) {
		systemPosition = pos;
	}

	public void setHomeInit(){
		io.resetHomePhase();
		io.resetFilter();
	}

	public boolean isAtPos() {
		if (systemState == ElevatorState.POSITION) {
			return switch (systemPosition) {
				case L1 -> io.isAtPosition(ElevatorConstants.ELEVATOR_L1_POSITION);
				case L2 -> io.isAtPosition(ElevatorConstants.ELEVATOR_L2_POSITION);
				case L3 -> io.isAtPosition(ElevatorConstants.ELEVATOR_L3_POSITION);
				case L4 -> io.isAtPosition(ElevatorConstants.ELEVATOR_L4_POSITION);
				case LOW_ALGAE -> io.isAtPosition(ElevatorConstants.ELEVATOR_LOW_ALGAE_POSITION);
				case HIGH_ALGAE -> io.isAtPosition(ElevatorConstants.ELEVATOR_HIGH_ALGAE_POSITION);
				case PROCESSOR -> io.isAtPosition(ElevatorConstants.ELEVATOR_PROCESSOR_POSITION);
				case STATION -> io.isAtPosition(ElevatorConstants.ELEVATOR_STATION_POSITION);
				default -> false;
			};
		} else {
			systemPosition = ElevatorPosition.NULL;
			return false;
		}
	}

	/* System States */
	public enum ElevatorState {
		IDLE,
		HOMING,
		HOLD,
		MANUALUPWARD,
		MANUTALDOWNWARD,
		POSITION
	}

	public enum ElevatorPosition {
		NULL,
		L1,
		L2,
		L3,
		L4,
		LOW_ALGAE,
		HIGH_ALGAE,
		PROCESSOR,
		STATION,
	}
}
