// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.rollers.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class HighAlgaeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RollerSubsystem m_rollerSubsystem;

  private final PivotSubsystem m_pivotSubsystem;

  private final ElevatorSubsystem m_elevatorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HighAlgaeCommand(
      RollerSubsystem rollerSubsystem,
      PivotSubsystem pivotSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    this.m_rollerSubsystem = rollerSubsystem;
    this.m_pivotSubsystem = pivotSubsystem;
    this.m_elevatorSubsystem = elevatorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollerSubsystem, pivotSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.resetFilter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.highAlgae();
    m_pivotSubsystem.highAlgae();
    if (m_pivotSubsystem.isAtPosition(PivotConstants.highAlgaePosition)
        && m_elevatorSubsystem.isAbovePosition(ElevatorConstants.highAlgaePosition)) {
      m_rollerSubsystem.intakeAlgaefromReef();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.setHoldAlgae(true);
    m_rollerSubsystem.setHoldAlgae(true);
    m_elevatorSubsystem.shutDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
