package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {


    private SuperStructureState systemState = SuperStructureState.STARTING_CONFIG;

    private enum SuperStructureState {
        STARTING_CONFIG,
        INITIALIZE,
        HOME,
        L1,
        L2,
        L3,
        L4,
        LOW_ALGAE,
        HIGH_ALGAE,
        PROCESSOR,
        BARGE,
        STATION,
        MANUAL
    }

    public SuperStructure() {

    }

    @Override
    public void periodic() {
        updateStateMachine();
    }

    public void updateStateMachine() {

    }

}
