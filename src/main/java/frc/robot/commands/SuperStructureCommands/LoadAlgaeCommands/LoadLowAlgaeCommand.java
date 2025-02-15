package frc.robot.commands.SuperStructureCommands.LoadAlgaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;

public class LoadLowAlgaeCommand extends Command {
    private SuperStructure m_superStructure;

    public LoadLowAlgaeCommand(SuperStructure superStructure) {
        m_superStructure = superStructure;
        addRequirements(m_superStructure);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_superStructure.requestLoadAlgae(SuperStructure.superStructurePosition.LOW_ALGAE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_superStructure.requestEnd();
    }
}
