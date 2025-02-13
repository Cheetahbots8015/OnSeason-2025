package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.*;

public class ElevatorTestCommand extends Command {
  private ElevatorSystem m_elevator;

  public ElevatorTestCommand(ElevatorSystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.testCommand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
