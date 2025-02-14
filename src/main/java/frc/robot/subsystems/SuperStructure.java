package frc.robot.subsystems;

import static frc.robot.util.PositionUtil.super2elevator;

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
  private superStructureState nextSystemState = superStructureState.IDLE;
  private superStructurePosition systemPosition = superStructurePosition.NULL;

  public SuperStructure(ElevatorSystem ele, RollerSystem roll, Vision vis, PivotSystem piv) {
    this.elevator = ele;
    this.roller = roll;
    this.vision = vis;
    this.pivot = piv;

    // nextSystemState = superStructureState.HOMING;
  }

  @Override
  public void periodic() {
    updateStateMachine();

    if (DriverStation.isDisabled()) {
      disabledCommands();
    }
  }

  public void updateStateMachine() {
    if (systemState != nextSystemState) {
      systemState = nextSystemState;
    }

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
      case LOAD:
        setLoad();
        break;
      case SHOOT:
        setShotReef();
        break;
      case POSITION:
        setPosition();
        break;
    }
  }

  /* public request from commands */
  public void requestHome() {
    if (!isHomed()) {
      nextSystemState = superStructureState.HOMING;
    } else {
      nextSystemState = superStructureState.IDLE;
    }
  }

  public void requestManual() {
    nextSystemState = superStructureState.MANUAL;
  }

  public void requestShootReef(superStructurePosition pos) {
    systemPosition = pos;
    if (isShot()) {
      nextSystemState = superStructureState.IDLE;
    } else if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else if (isAtPosition()) {
      nextSystemState = superStructureState.SHOOT;
    }
  }

  public void requestLoad() {
    systemPosition = superStructurePosition.STATION;
    if (isLoaded()) {
      nextSystemState = superStructureState.IDLE;
    } else if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else if (isAtPosition()) {
      nextSystemState = superStructureState.LOAD;
    }
  }

  public void requestShootProcessor() {
    systemPosition = superStructurePosition.PROCESSOR;
    if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else if (isAtPosition()) {
      nextSystemState = superStructureState.SHOOT;
    }
  }

  public void requestLoadAlgae(superStructurePosition pos) {
    systemPosition = pos;
    if (isLoaded()) {
      nextSystemState = superStructureState.IDLE;
    } else if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else if (isAtPosition()) {
      nextSystemState = superStructureState.LOAD;
    }
  }

  public void requestEnd() {
    nextSystemState = superStructureState.IDLE;
  }

  /* private request to other subsystems */
  private void setHome() {
    elevator.setRequest(ElevatorSystem.ElevatorRequest.HOME);
  }

  private void setHold() {
    elevator.setRequest(ElevatorSystem.ElevatorRequest.NULL);
  }

  private void setManual() {
    elevator.setRequest(ElevatorSystem.ElevatorRequest.MANUAL);
  }

  private void setPosition() {
    elevator.setPositionTarget(super2elevator(systemPosition));
    elevator.setRequest(ElevatorSystem.ElevatorRequest.POSITION);
  }

  private void setLoad() {
  }

  private void setShotReef() {
  }

  /* private boolean from other subsystems*/
  private boolean isHomed() {
    return false;
  }

  private boolean isLoaded() {
    return false;
  }

  private boolean isShot() {
    return false;
  }

  private boolean isAtPosition() {
    return false;
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

  public enum superStructurePosition {
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
