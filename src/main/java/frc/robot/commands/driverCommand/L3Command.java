// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.rollers.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class L3Command extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevatorSubsystem;

  private final PivotSubsystem m_pivotSubsystem;
  private final RollerSubsystem m_rollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public L3Command(
      ElevatorSubsystem elevatorSubsystem,
      PivotSubsystem pivotSubsystem,
      RollerSubsystem rollerSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    m_rollerSubsystem = rollerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.resetFilter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.L3();
    m_pivotSubsystem.L3();
    if (m_elevatorSubsystem.isAtPosition(ElevatorConstants.ELEVATOR_L3_POSITION)
        && m_pivotSubsystem.isAtPosition(PivotConstants.PIVOT_L3_ANGLE)) {
      m_rollerSubsystem.L3();
    } else {
      m_rollerSubsystem.defaultIdleVelocity();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerSubsystem.defaultIdleVelocity();
    m_elevatorSubsystem.shutDown();
    m_pivotSubsystem.shutDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
