package frc.robot.commands.SuperStructureCommands.LoadFromStationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;

public class LoadFromStationCommand extends Command {
  private SuperStructure m_superStructure;

  public LoadFromStationCommand(SuperStructure superStructure) {
    m_superStructure = superStructure;
    addRequirements(m_superStructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_superStructure.requestLoad();
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
