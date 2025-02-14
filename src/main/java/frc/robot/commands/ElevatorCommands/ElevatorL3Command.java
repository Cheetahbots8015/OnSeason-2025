package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorSystem.ElevatorPositionTarget;
import frc.robot.subsystems.elevator.ElevatorSystem.ElevatorRequest;

public class ElevatorL3Command extends Command {
  private ElevatorSystem m_elevator;

  public ElevatorL3Command(ElevatorSystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.setRequest(ElevatorRequest.POSITION);
    m_elevator.setUsePositionDynamic(false);
    m_elevator.setPositionTarget(ElevatorPositionTarget.L3);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.setRequest(ElevatorRequest.NULL);
    m_elevator.setPositionTarget(ElevatorPositionTarget.NULL);
  }
}
