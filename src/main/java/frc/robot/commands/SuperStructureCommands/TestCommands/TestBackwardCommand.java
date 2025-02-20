package frc.robot.commands.SuperStructureCommands.TestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;

public class TestBackwardCommand extends Command {
  private SuperStructure m_superStructure;

  public TestBackwardCommand(SuperStructure superStructure) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    m_superStructure = superStructure;
    addRequirements(m_superStructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_superStructure.requestManualBackward();
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
