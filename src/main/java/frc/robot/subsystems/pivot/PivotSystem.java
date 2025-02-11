package frc.robot.subsystems.pivot;

public class PivotSystem {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PivotState systemState = PivotState.INITIALIZED;

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
    this.pivotIO = pivotIO;
  }

  public void updateStateMachine() {}
}
