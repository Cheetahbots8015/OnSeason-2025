// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class HomeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevatorSubsystem;

  private final PivotSubsystem m_pivotSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HomeCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setHasHomed(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.home();
    m_pivotSubsystem.home();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.shutDown();
    m_elevatorSubsystem.resetTimer();
    m_pivotSubsystem.hold();
    if (!interrupted) {
      m_elevatorSubsystem.resetOffset();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.getHasHomed() && m_pivotSubsystem.isAtPosition(0.0);
  }
}
