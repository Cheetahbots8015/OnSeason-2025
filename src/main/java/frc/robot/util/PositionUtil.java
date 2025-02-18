package frc.robot.util;

import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.elevator.ElevatorSystem;
import frc.robot.subsystems.pivot.PivotSystem;
import frc.robot.subsystems.rollers.RollerSystem;

public class PositionUtil {
    public static ElevatorSystem.ElevatorPosition super2elevator(
            SuperStructure.superStructurePosition pos) {
        switch (pos) {
            case L1 -> {
                return ElevatorSystem.ElevatorPosition.L1;
            }
            case L2 -> {
                return ElevatorSystem.ElevatorPosition.L2;
            }
            case L3 -> {
                return ElevatorSystem.ElevatorPosition.L3;
            }
            case L4 -> {
                return ElevatorSystem.ElevatorPosition.L4;
            }
            case LOW_ALGAE -> {
                return ElevatorSystem.ElevatorPosition.LOW_ALGAE;
            }
            case HIGH_ALGAE -> {
                return ElevatorSystem.ElevatorPosition.HIGH_ALGAE;
            }
            case PROCESSOR -> {
                return ElevatorSystem.ElevatorPosition.PROCESSOR;
            }
            case STATION -> {
                return ElevatorSystem.ElevatorPosition.STATION;
            }
            case LOLIPOP -> {
                return ElevatorSystem.ElevatorPosition.LOLIPOP;
            }
            default -> {
                return ElevatorSystem.ElevatorPosition.NULL;
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
            case LOLIPOP -> {
                return RollerSystem.RollerPosition.LOLIPOP;
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
