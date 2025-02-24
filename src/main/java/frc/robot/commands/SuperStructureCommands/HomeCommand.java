package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;


public class HomeCommand extends Command {
	private SuperStructure m_superStructure;

	public HomeCommand(SuperStructure superStructure) {
		m_superStructure = superStructure;
		addRequirements(m_superStructure);
	}

	@Override
	public void initialize() {
		m_superStructure.requestHomeInit();
	}

	@Override
	public void execute() {
		m_superStructure.requestHome();
	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		m_superStructure.requestEnd();
	}
}
