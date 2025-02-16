package frc.robot.commands.SuperStructureCommands.ShootReefCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;

public class ShootLeftReefL4Command extends Command {
  private SuperStructure m_superStructure;

  public ShootLeftReefL4Command(SuperStructure superStructure) {
    this.m_superStructure = superStructure;
    addRequirements(m_superStructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_superStructure.requestShootReef(SuperStructure.superStructurePosition.L4, true);
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
