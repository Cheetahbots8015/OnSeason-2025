package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class PivotSystem extends SubsystemBase {
  private final PivotIO io;
  private final String name;
  private final Alert disconnected;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final SysIdRoutine sysIdRoutinePivot;
  private final SysIdRoutine routineToApply;

  private PivotState pivotState = PivotState.IDLE;
  private PivotPosition pivotPosition = PivotPosition.NULL;
  private PivotState nextPivotState = PivotState.IDLE;

  private boolean isAlgae = false;

  public PivotSystem(String name, PivotIO pivotIO) {
    this.name = name;
    this.io = pivotIO;
    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);

    this.sysIdRoutinePivot =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(1.5),
                Seconds.of(3.5),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  io.setVoltage(volts.magnitude() * 0.5);
                },
                null,
                this));
    this.routineToApply = sysIdRoutinePivot;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    Logger.recordOutput(name, this.getPivotState());
    disconnected.set(!inputs.connected);

    if (DriverStation.isDisabled()) {
      nextPivotState = PivotState.IDLE;
    }
    updateStateMachine();
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public void setPivotState(PivotState state) {
    nextPivotState = state;
  }

  public void setPivotPosition(PivotPosition position) {
    pivotPosition = position;
  }

  public void setHoldAlgae() {
    isAlgae = true;
  }

  public void unsetHoldAlgae() {
    isAlgae = false;
  }

  public boolean isAtPosition() {
    return switch (pivotPosition) {
      case L1 -> io.isAtPosition(PivotConstants.PIVOT_L1_ANGLE);
      case L2 -> io.isAtPosition(PivotConstants.PIVOT_L2_ANGLE);
      case L3 -> io.isAtPosition(PivotConstants.PIVOT_L3_ANGLE);
      case L4 -> io.isAtPosition(PivotConstants.PIVOT_L4_ANGLE);
      case HIGHALGAE -> io.isAtPosition(PivotConstants.PIVOT_HIGHALGAE_ANGLE);
      case LOWALGAE -> io.isAtPosition(PivotConstants.PIVOT_LOWALGAE_ANGLE);
      case PROCESSOR -> io.isAtPosition(PivotConstants.PIVOT_PROCESSOR_ANGLE);
      default -> false;
    };
  }

  public void updateStateMachine() {
    if (pivotState != nextPivotState) {
      pivotState = nextPivotState;
    }

    switch (pivotState) {
      case IDLE:
        if (isAlgae) {
          io.setAngle(PivotConstants.PIVOT_ALGAE_HOLD_ANGLE);
        } else {
          io.setAngle(PivotConstants.PIVOT_CORAL_HOLD_ANGLE);
        }
        break;
      case POSITION:
        switch (pivotPosition) {
          case NULL:
            io.hold();
            break;
          case L1:
            io.setAngle(PivotConstants.PIVOT_L1_ANGLE);
            break;
          case L2:
            io.setAngle(PivotConstants.PIVOT_L2_ANGLE);
            break;
          case L3:
            io.setAngle(PivotConstants.PIVOT_L3_ANGLE);
            break;
          case L4:
            io.setAngle(PivotConstants.PIVOT_L4_ANGLE);
            break;
          case HIGHALGAE:
            io.setAngle(PivotConstants.PIVOT_HIGHALGAE_ANGLE);
            break;
          case LOWALGAE:
            io.setAngle(PivotConstants.PIVOT_LOWALGAE_ANGLE);
            break;
          case PROCESSOR:
            io.setAngle(PivotConstants.PIVOT_PROCESSOR_ANGLE);
            break;
        }
        break;
      case MANUALFORWARD:
        io.setVoltage(PivotConstants.PIVOT_MANUAL_FORWARD_VOLTAGE);
        break;
      case MANUALREVERSE:
        io.setVoltage(PivotConstants.PIVOT_MANUAL_REVERSE_VOLTAGE);
        break;
    }
  }

  public enum PivotState {
    IDLE,
    POSITION,
    MANUALFORWARD,
    MANUALREVERSE
  }

  public enum PivotPosition {
    NULL,
    L1,
    L2,
    L3,
    L4,
    HIGHALGAE,
    LOWALGAE,
    PROCESSOR,
  }
}
