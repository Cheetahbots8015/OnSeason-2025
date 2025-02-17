package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.generated.PivotConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    public boolean isAtPosition() {
        return switch (pivotPosition) {
            case L1 -> Math.abs((inputs.position % 1.0) - PIVOT_L1_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case L2 -> Math.abs((inputs.position % 1.0) - PIVOT_L2_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case L3 -> Math.abs((inputs.position % 1.0) - PIVOT_L3_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case L4 -> Math.abs((inputs.position % 1.0) - PIVOT_L4_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case HIGHALGAE -> Math.abs((inputs.position % 1.0) - PIVOT_HIGHALGAE_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case LOWALGAE -> Math.abs((inputs.position % 1.0) - PIVOT_LOWALGAE_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case LOLIPOP -> Math.abs((inputs.position % 1.0) - PIVOT_LOLIPOP_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            case PROCESSOR -> Math.abs((inputs.position % 1.0) - PIVOT_PROCESSOR_HEIGHT) < PIVOT_POSITION_TOLERANCE;
            default -> false;
        };
    }

    public void updateStateMachine() {
        if (pivotState != nextPivotState) {
            pivotState = nextPivotState;
        }

        switch (pivotState) {
            case IDLE:
                io.hold(inputs.position);
                break;
            case POSITION:
                double offset = inputs.s1Position - (inputs.s1Position % 1.0);
                switch (pivotPosition) {
                    case NULL:
                        io.hold(inputs.position);
                        break;
                    case L1:
                        io.setAngle(PIVOT_L1_HEIGHT, offset);
                        break;
                    case L2:
                        io.setAngle(PIVOT_L2_HEIGHT, offset);
                        break;
                    case L3:
                        io.setAngle(PIVOT_L3_HEIGHT, offset);
                        break;
                    case L4:
                        io.setAngle(PIVOT_L4_HEIGHT, offset);
                        break;
                    case HIGHALGAE:
                        io.setAngle(PIVOT_HIGHALGAE_HEIGHT, offset);
                        break;
                    case LOWALGAE:
                        io.setAngle(PIVOT_LOWALGAE_HEIGHT, offset);
                        break;
                    case LOLIPOP:
                        io.setAngle(PIVOT_LOLIPOP_HEIGHT, offset);
                        break;
                    case PROCESSOR:
                        io.setAngle(PIVOT_PROCESSOR_HEIGHT, offset);
                        break;
                }
                break;
            case MANUALFORWARD:
                io.setVoltage(PIVOT_MANUAL_FORWARD_VOLTAGE);
                break;
            case MANUALREVERSE:
                io.setVoltage(PIVOT_MANUAL_REVERSE_VOLTAGE);
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
        LOLIPOP,
        PROCESSOR,
    }
}
