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
import frc.robot.generated.RollerConstants;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
    protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
    private final String name;
    private final RollerSystemIO io;
    private final Alert disconnected;

    private boolean isLoaded = false;
    private boolean isShot = true;

    private RollerState systemState = RollerState.IDLE;
    private RollerPosition systemPosition = RollerPosition.NULL;
    private RollerState nextSystemState = RollerState.IDLE;

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

    public void setSystemState(RollerState state) {
        nextSystemState = state;
    }

    private void updateStateMachine() {
        if (systemState != nextSystemState) {
            systemState = nextSystemState;
        }

        if (isLoaded && !io.isCanRangeTriggered()) {
            isLoaded = false;
            isShot = true;
        }
        if (io.isCanRangeTriggered()) {
            isShot = false;
            isLoaded = true;
        }


        switch (systemState) {
            case IDLE:
                break;
            case LOAD:
                io.runVolts(RollerConstants.ROLLER_LOAD_VOLTAGE);
                break;
            case SHOOT:
                switch (systemPosition) {
                    case L1 -> io.runVolts(RollerConstants.ROLLER_L1_VOLTAGE);
                    case L2 -> io.runVolts(RollerConstants.ROLLER_L2_VOLTAGE);
                    case L3 -> io.runVolts(RollerConstants.ROLLER_L3_VOLTAGE);
                    case L4 -> io.runVolts(RollerConstants.ROLLER_L4_VOLTAGE);
                    case LOLIPOP -> io.runVolts(RollerConstants.ROLLER_LOLIPOP_VOLTAGE);
                    case NULL -> io.runVolts(0);
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

    public void setSystemPosition(RollerPosition position) {
        systemPosition = position;
    }

    public boolean getLoaded() {
        return isLoaded;
    }

    public boolean getShot() {
        return isShot;
    }

    public enum RollerState {
        IDLE,
        LOAD,
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
        LOLIPOP,
    }
}
