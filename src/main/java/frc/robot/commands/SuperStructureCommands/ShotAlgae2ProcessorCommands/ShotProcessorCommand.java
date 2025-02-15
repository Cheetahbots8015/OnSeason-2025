package frc.robot.commands.SuperStructureCommands.ShotAlgae2ProcessorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;

public class ShotProcessorCommand extends Command {
    private SuperStructure m_superStructure;

    public ShotProcessorCommand(SuperStructure superStructure) {
        m_superStructure = superStructure;
        addRequirements(m_superStructure);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_superStructure.requestShootProcessor();
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
