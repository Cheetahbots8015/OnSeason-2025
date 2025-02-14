package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSystem extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PivotState systemState = PivotState.INITIALIZED;

  private boolean requestHome = false;
  private boolean homed = false;
  private boolean requestPosition = false;
  private String positionString = "";

  private enum PivotState {
    INITIALIZED,
    HOME,
    PROCESSOR,
    BARGE,
    L1,
    L2,
    L3,
    L4,
    STATION,
    REEF
  }

  public PivotSystem(PivotIO pivotIO) {
    this.io = pivotIO;
  }

    private void home() {
    }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    updateStateMachine();
  }

  public PivotState getPivotState() {
    return systemState;
  }

  public void updateStateMachine() {
      if (systemState == PivotState.INITIALIZED) {
      requestHome = true;
    }

      if (requestHome) {
      home();
      systemState = PivotState.HOME;
      homed = true;
      requestHome = false;
    }

      if (requestPosition && homed) {
          switch (positionString) {
        case "L1":
          systemState = PivotState.L1;
          break;

        case "L2":
          systemState = PivotState.L2;
          break;

        case "L3":
          systemState = PivotState.L3;
          break;

        case "L4":
          systemState = PivotState.L4;
          break;

        case "Station":
            systemState = PivotState.STATION;
            break;

        case "Reef":
            systemState = PivotState.REEF;
            break;

        case "Barge":
            systemState = PivotState.BARGE;
            break;

        case "Processor":
            systemState = PivotState.PROCESSOR;
            break;

        default:
          break;
      }
    }
  }
}
