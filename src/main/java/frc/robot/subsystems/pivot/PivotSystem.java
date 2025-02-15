package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSystem extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private PivotState pivotState = PivotState.IDLE;
  private PivotPosition pivotPosition = PivotPosition.NULL;
  private PivotState nextPivotState = PivotState.IDLE;

  private enum PivotState {
    IDLE,
    POSITION,
    MANUAL
  }

  public enum PivotPosition {
    NULL,
    L1,
    L2,
    L3,
    L4,
    ALGAE,
    PROCESSOR,
  }

  public PivotSystem(PivotIO pivotIO) {
    this.io = pivotIO;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    updateStateMachine();
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public void updateStateMachine() {
      
  }
}
