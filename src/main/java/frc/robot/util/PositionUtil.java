package frc.robot.util;

import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.elevator.ElevatorSystem;

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
}
