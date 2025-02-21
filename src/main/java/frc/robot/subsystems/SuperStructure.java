package frc.robot.subsystems;

import static frc.robot.util.PositionUtil.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorSystem;
import frc.robot.subsystems.pivot.PivotSystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final ElevatorSystem elevator;
  private RollerSystem roller;
  private Vision vision;
  private PivotSystem pivot;

  private boolean isLeft = true;

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
    Logger.recordOutput("Super Structure", this.getSystemState());
    Logger.recordOutput("isLoaded", this.isCoralLoaded());
    Logger.recordOutput("isPosition", this.isAtPosition());
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
      case MANUALFORWARD:
        setManualForward();
        break;
      case MANUALBACKWARD:
        setManualBackward();
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

  public void requestManualForward() {
    nextSystemState = superStructureState.MANUALFORWARD;
  }

  public void requestManualBackward() {
    nextSystemState = superStructureState.MANUALBACKWARD;
  }

  public void requestShootReef(superStructurePosition pos, boolean isLeft) {
    systemPosition = pos;
    this.isLeft = isLeft;
    if (!isCoralLoaded()) {
      nextSystemState = superStructureState.IDLE;
    } else if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else {
      nextSystemState = superStructureState.SHOOT;
    }
  }

  public void requestLoad() {
    systemPosition = superStructurePosition.STATION;
    if (isCoralLoaded()) {
      nextSystemState = superStructureState.IDLE;
    } else if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else {
      nextSystemState = superStructureState.LOAD;
    }
  }

  public void requestShootProcessor() {
    systemPosition = superStructurePosition.PROCESSOR;
    if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else {
      nextSystemState = superStructureState.SHOOT;
    }
  }

  public void requestLoadAlgae(superStructurePosition pos) {
    systemPosition = pos;
    if (!isAtPosition()) {
      nextSystemState = superStructureState.POSITION;
    } else {
      nextSystemState = superStructureState.LOAD;
    }
  }

  public void requestEnd() {
    nextSystemState = superStructureState.IDLE;
    systemPosition = superStructurePosition.NULL;
  }

  private superStructureState getSystemState() {
    return systemState;
  }

  /* private request to other subsystems */
  private void setHome() {
    elevator.setElevatorState(ElevatorSystem.ElevatorState.HOMING);
  }

  private void setHold() {
    elevator.setElevatorState(ElevatorSystem.ElevatorState.IDLE);
    pivot.setPivotState(PivotSystem.PivotState.IDLE);
    roller.setSystemState(RollerSystem.RollerState.IDLE);
  }

  private void setManualForward() {
    elevator.setElevatorState(ElevatorSystem.ElevatorState.MANUALUPWARD);
  }

  private void setManualBackward() {
    elevator.setElevatorState(ElevatorSystem.ElevatorState.MANUTALDOWNWARD);
  }

  private void setPosition() {
    elevator.setElevatorPosition(super2elevator(systemPosition));
    elevator.setElevatorState(ElevatorSystem.ElevatorState.POSITION);

    pivot.setPivotPosition(super2pivot(systemPosition));
    pivot.setPivotState(PivotSystem.PivotState.POSITION);
  }

  private void setLoad() {
    roller.setSystemState(RollerSystem.RollerState.LOAD);
  }

  private void setShotReef() {
    roller.setSystemPosition(super2roller(systemPosition));
    roller.setSystemState(RollerSystem.RollerState.SHOOT);
  }

  /* private boolean from other subsystems*/
  private boolean isHomed() {
    return elevator.getHomed();
  }

  private boolean isCoralLoaded() {
    return roller.getLoaded();
  }

  private boolean isAtPosition() {
    return elevator.isAtPos(); // pivot.isAtPosition() &&
  }

  // commands when disabled
  private void disabledCommands() {
    systemState = superStructureState.IDLE;
    systemPosition = superStructurePosition.NULL;
  }

  private enum superStructureState {
    IDLE,
    HOMING,
    MANUALFORWARD,
    MANUALBACKWARD,
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
    STATION,
    LOLIPOP
  }
}
