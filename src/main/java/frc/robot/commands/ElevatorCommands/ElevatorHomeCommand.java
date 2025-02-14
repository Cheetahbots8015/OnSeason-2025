package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorSystem.ElevatorRequest;

public class ElevatorHomeCommand extends Command {
  private ElevatorSystem m_elevator;

  public ElevatorHomeCommand(ElevatorSystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.setRequest(ElevatorRequest.HOME);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.setRequest(ElevatorRequest.NULL);
  }
}
