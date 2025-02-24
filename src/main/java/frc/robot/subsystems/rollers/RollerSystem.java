// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RollerConstants;
import frc.robot.util.MagicTimer;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
	protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
	private final String name;
	private final RollerSystemIO io;
	private final Alert disconnected;

	private RollerState systemState = RollerState.IDLE;
	private RollerPosition systemPosition = RollerPosition.NULL;
	private RollerState nextSystemState = RollerState.IDLE;

	private MagicTimer rollerTimer = new MagicTimer();
	private boolean loadFinsihed = false;

	public RollerSystem(String name, RollerSystemIO io) {
		this.name = name;
		this.io = io;
		disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(name, inputs);
		Logger.recordOutput(name, this.getSystemState());
		disconnected.set(!inputs.connected);

		if (DriverStation.isDisabled()) {
			nextSystemState = RollerState.IDLE;
		}
		updateStateMachine();
	}

	public RollerState getSystemState() {
		return systemState;
	}

	private void updateStateMachine() {
		if (systemState != nextSystemState) {
			systemState = nextSystemState;
		}

		switch (systemState) {
			case IDLE:
				io.stop();
				break;

			case HOLDCORAL:
				io.runVelocity(RollerConstants.ROLLER_CORAL_IDLE_VELOCITY);
				break;

			case HOLDALGAE:
				io.runVelocity(RollerConstants.ROLLER_ALGAE_IDLE_VELOCITY);
				break;

			case LOADCORAL:
				if (io.isCanRangeTriggered()) {
					rollerTimer.startTimer();
					if (rollerTimer.getTimePassedSec() > RollerConstants.ROLLER_STATION_TIME) {
						loadFinsihed = true;
						io.runVelocity(RollerConstants.ROLLER_CORAL_IDLE_VELOCITY);
					} else {
						loadFinsihed = false;
						io.runVelocity(RollerConstants.ROLLER_LOAD_CORAL_VELOCITY);
					}
				} else {
					loadFinsihed = false;
					rollerTimer.resetTimer();
					io.runVelocity(RollerConstants.ROLLER_LOAD_CORAL_VELOCITY);
				}
				break;

			case LOADALGAE:
				io.runVelocity(RollerConstants.ROLLER_LOAD_ALGAE_VELOCITY);
				break;

			case SHOOT:
				loadFinsihed = false;
				switch (systemPosition) {
					case L1 -> io.runVelocity(RollerConstants.ROLLER_L1_VELOCITY);
					case L2 -> io.runVelocity(RollerConstants.ROLLER_L2_VELOCITY);
					case L3 -> io.runVelocity(RollerConstants.ROLLER_L3_VELOCITY);
					case L4 -> io.runVelocity(RollerConstants.ROLLER_L4_VELOCITY);
					case PROCESSOR -> io.runVelocity(RollerConstants.ROLLER_PROCESSOR_VELOCITY);
					case NULL -> io.runVelocity(0);
				}
				break;

			case MANUALFORWARD:
				io.runVolts(RollerConstants.ROLLER_MANUALFORWARD_VOLTAGE);
				break;

			case MANUALINVERT:
				io.runVolts(RollerConstants.ROLLER_MANUALINVERT_VOLTAGE);
				break;
		}
	}

	public void setRollerState(RollerState state) {
		nextSystemState = state;
	}

	public void setRollerPosition(RollerPosition position) {
		systemPosition = position;
	}

	public boolean getLoaded() {
		return io.isCanRangeTriggered() && loadFinsihed;
	}

	public enum RollerState {
		IDLE,
		LOADCORAL,
		LOADALGAE,
		HOLDCORAL,
		HOLDALGAE,
		SHOOT,
		MANUALINVERT,
		MANUALFORWARD,
	}

	public enum RollerPosition {
		NULL,
		L1,
		L2,
		L3,
		L4,
		PROCESSOR,
	}
}
