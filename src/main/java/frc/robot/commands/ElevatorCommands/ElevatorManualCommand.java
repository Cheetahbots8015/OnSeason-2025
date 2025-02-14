package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorSystem.ElevatorRequest;

public class ElevatorManualCommand extends Command {
  private ElevatorSystem m_elevator;

  public ElevatorManualCommand(ElevatorSystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.setRequest(ElevatorRequest.MANUAL);
    m_elevator.setManualVoltage(1.0);
    m_elevator.setUseManualDynamic(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.setRequest(ElevatorRequest.NULL);
    m_elevator.setManualVoltage(0.0);
    m_elevator.setUseManualDynamic(false);
  }
}
