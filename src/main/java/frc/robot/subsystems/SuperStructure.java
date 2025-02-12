package frc.robot.subsystems;

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

    private SuperStructureState systemState = SuperStructureState.INITIALIZE;

    private boolean homed = false;
    private boolean loaded = false;
    private boolean atSetPoint = false;

    private boolean requestHome = false;
    private boolean requestShoot = false;
    private boolean requestLoad = false;
    private boolean requestProcessor = false;
    private boolean requestManual = false;
    private String requestAlgae = null;
    private String requestPosition = null;

    public SuperStructure(ElevatorSystem ele, RollerSystem roll, Vision vis, PivotSystem piv) {
        this.elevator = ele;
        this.roller = roll;
        this.vision = vis;
        this.pivot = piv;
    }

    @Override
    public void periodic() {
        updateStateMachine();
        periodicalUpdate();
    }

    public void updateStateMachine() {
        if (requestManual) {
            systemState = SuperStructureState.MANUAL;
            homed = false;
        }

        if (systemState == SuperStructureState.INITIALIZE) {
            requestHome = true;
        }

        if (requestHome) {
            home();
            if (homed) {
                requestHome = false;
                systemState = SuperStructureState.HOME;
            }
        }

        if (requestPosition != null && homed && !atSetPoint) {
            switch (requestPosition) {
                case "L1":
                    set2Pos("L1");
                    systemState = SuperStructureState.L1;
                case "L2":
                    set2Pos("L2");
                    systemState = SuperStructureState.L2;
                case "L3":
                    set2Pos("L3");
                    systemState = SuperStructureState.L3;
                case "L4":
                    set2Pos("L4");
                    systemState = SuperStructureState.L4;
                case "LOW_ALGAE":
                    set2Pos("LOW_ALGAE");
                    systemState = SuperStructureState.LOW_ALGAE;
                case "HIGH_ALGAE":
                    set2Pos("HIGH_ALGAE");
                    systemState = SuperStructureState.HIGH_ALGAE;
                case "PROCESSOR":
                    set2Pos("PROCESSOR");
                    systemState = SuperStructureState.PROCESSOR;
                case "STATION":
                    set2Pos("STATION");
                    systemState = SuperStructureState.STATION;
                default:
                    break;
            }
        }

        if (requestPosition != null && loaded && requestShoot && atSetPoint) {
            switch (systemState) {
                case L1:
                    shoot("L1");
                case L2:
                    shoot("L2");
                case L3:
                    shoot("L3");
                case L4:
                    shoot("L4");
                default:
                    break;
            }
        }

        if (requestPosition != null && !loaded && requestLoad && atSetPoint) {
            load("Station");
        }

        if (requestPosition != null && requestProcessor && atSetPoint) {
            shootProcessor();
        }

        if (requestPosition != null && !loaded && requestAlgae != null && atSetPoint) {
            switch (systemState) {
                case HIGH_ALGAE -> getAlgae("HIGH");
                case LOW_ALGAE -> getAlgae("LOW");
                default -> {
                    break;
                }
            }
        }
    }

    /* commands to other subsystems */
    private void home() {
    }

    private void shoot(String pos) {
    }

    private void set2Pos(String pos) {
    }

    private void load(String pos) {
    }

    private void shootProcessor() {
    }

    private void getAlgae(String pos) {

    }

    /* get state from other subsystems */
    private void gethomed() {
        this.homed = true;
    }

    private void getSetPoint() {
        this.atSetPoint = true;
    }

    private void getLoaded() {
        this.loaded = true;
    }

    private void periodicalUpdate() {
        gethomed();
        getSetPoint();
        getLoaded();
    }

    /* public request */
    public void requestManual() {
        this.requestManual = true;
    }

    public void requestLoad() {
        this.requestLoad = true;
    }

    public void requestShoot() {
        this.requestShoot = true;
    }

    public void requestPosition(String pos) {
        this.requestPosition = pos;
    }

    public void requestAlgae(String pos) {
        this.requestAlgae = pos;
    }

    private void resetRequest() { //TODO: need to at when to reset
        requestHome = false;
        requestShoot = false;
        requestLoad = false;
        requestProcessor = false;
        requestManual = false;
        requestPosition = null;
    }

    private enum SuperStructureState {
        INITIALIZE,
        HOME,
        L1,
        L2,
        L3,
        L4,
        LOW_ALGAE,
        HIGH_ALGAE,
        PROCESSOR,
        STATION,
        MANUAL
    }
}
