package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSystem;
import frc.robot.subsystems.pivot.PivotSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.vision.Vision;

public class SuperStructure extends SubsystemBase {
    private ElevatorSystem elevator;
    private RollerSystem roller;
    private Vision vision;
    private PivotSystem pivot;

    private superStructureState systemState = superStructureState.IDLE;
    private superStructurePosition systemPosition = superStructurePosition.NULL;

    public SuperStructure(ElevatorSystem ele, RollerSystem roll, Vision vis, PivotSystem piv) {
        this.elevator = ele;
        this.roller = roll;
        this.vision = vis;
        this.pivot = piv;
    }

    @Override
    public void periodic() {
        updateStateMachine();

        if (DriverStation.isDisabled()) {
            disabledCommands();
        }
    }

    public void updateStateMachine() {
        switch (systemState) {
            case IDLE:
                setHold();
                break;

            case HOMING:
                setHome();
                break;

            case MANUAL:
                setManual();
                break;

            case POSITION:
                setPosition();
                if (isAtPosition(systemPosition)) {
                    switch (systemPosition) {
                        case HIGH_ALGAE, STATION, LOW_ALGAE -> systemState = superStructureState.LOAD;
                        case L1, L2, L3, L4, PROCESSOR -> systemState = superStructureState.SHOOT;
                    }
                }
                break;

            case LOAD:
                setLoad();
        }
    }

    /* commands to other subsystems */
    private void setHold() {
    }

    private void setHome() {
        if (isHomed()) {
            systemState = superStructureState.IDLE;
        }
    }

    private void setManual() {
    }

    private void setPosition() {
        switch (systemPosition){
            default -> setHold();
        }
    }

    private void setLoad() {
    }

    /* boolean to confirm state changed */
    private boolean isHomed() {
        return false;
    }

    private boolean isAtPosition(superStructurePosition pos) {
        return false;
    }

    private boolean isLoaded() {
        return false;
    }

    private boolean isShot() {
        return false;
    }

    /* request to change the state */
    public void requestHome() {
        systemState = superStructureState.HOMING;
    }

    public void requestManual() {
        systemState = superStructureState.MANUAL;
    }

    private void requestPosition(superStructurePosition pos) {
        systemState = superStructureState.POSITION;
        systemPosition = pos;
    }


    // commands when disabled
    private void disabledCommands() {
    }


    private enum superStructureState {
        IDLE,
        HOMING,
        MANUAL,
        LOAD,
        SHOOT,
        POSITION
    }

    private enum superStructurePosition {
        NULL,
        HOME,
        L1,
        L2,
        L3,
        L4,
        LOW_ALGAE,
        HIGH_ALGAE,
        PROCESSOR,
        STATION
    }
}
