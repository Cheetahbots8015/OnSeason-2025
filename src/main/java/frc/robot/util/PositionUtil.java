package frc.robot.util;

import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.elevator.ElevatorSystem;
import frc.robot.subsystems.pivot.PivotSystem;
import frc.robot.subsystems.rollers.RollerSystem;

public class PositionUtil {
    public static ElevatorSystem.ElevatorPositionTarget super2elevator(
            SuperStructure.superStructurePosition pos) {
        switch (pos) {
            case L1 -> {
                return ElevatorSystem.ElevatorPositionTarget.L1;
            }
            case L2 -> {
                return ElevatorSystem.ElevatorPositionTarget.L2;
            }
            case L3 -> {
                return ElevatorSystem.ElevatorPositionTarget.L3;
            }
            case L4 -> {
                return ElevatorSystem.ElevatorPositionTarget.L4;
            }
            case LOW_ALGAE -> {
                return ElevatorSystem.ElevatorPositionTarget.LOW_ALGAE;
            }
            case HIGH_ALGAE -> {
                return ElevatorSystem.ElevatorPositionTarget.HIGH_ALGAE;
            }
            case PROCESSOR -> {
                return ElevatorSystem.ElevatorPositionTarget.PROCESSOR;
            }
            case HOME -> {
                return ElevatorSystem.ElevatorPositionTarget.HOME;
            }
            case STATION -> {
                return ElevatorSystem.ElevatorPositionTarget.STATION;
            }
            default -> {
                return ElevatorSystem.ElevatorPositionTarget.NULL;
            }
        }
    }

    public static RollerSystem.RollerPosition super2roller(
            SuperStructure.superStructurePosition pos) {
        switch (pos) {
            case L1 -> {
                return RollerSystem.RollerPosition.L1;
            }
            case L2 -> {
                return RollerSystem.RollerPosition.L2;
            }
            case L3 -> {
                return RollerSystem.RollerPosition.L3;
            }
            case L4 -> {
                return RollerSystem.RollerPosition.L4;
            }
            case STATION -> {
                return RollerSystem.RollerPosition.STATION;
            }
            default -> {
                return RollerSystem.RollerPosition.NULL;
            }
        }
    }

    public static PivotSystem.PivotPosition super2pivot(SuperStructure.superStructurePosition pos) {
        switch (pos) {
            case L1 -> {
                return PivotSystem.PivotPosition.L1;
            }
            case L2 -> {
                return PivotSystem.PivotPosition.L2;
            }
            case L3 -> {
                return PivotSystem.PivotPosition.L3;
            }
            case L4 -> {
                return PivotSystem.PivotPosition.L4;
            }
            case LOW_ALGAE -> {
                return PivotSystem.PivotPosition.LOWALGAE;
            }
            case HIGH_ALGAE -> {
                return PivotSystem.PivotPosition.HIGHALGAE;
            }
            case LOLIPOP -> {
                return PivotSystem.PivotPosition.LOLIPOP;
            }
            case PROCESSOR -> {
                return PivotSystem.PivotPosition.PROCESSOR;
            }
            default -> {
                return PivotSystem.PivotPosition.NULL;
            }
        }
    }
}
