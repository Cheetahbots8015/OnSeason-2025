// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class TakeAlgaeHomeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RollerSubsystem m_rollerSubsystem;

  private final PivotSubsystem m_pivotSubsystem;

  private final ElevatorSubsystem m_elevatorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TakeAlgaeHomeCommand(
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.home();
    m_pivotSubsystem.algaeHome();
    m_rollerSubsystem.shutDown();
    m_pivotSubsystem.report();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.hold();
    m_elevatorSubsystem.lockVolts();
    m_rollerSubsystem.shutDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.isAtPosition(0.0)
        && m_pivotSubsystem.isAtPosition(PivotConstants.algaeHome);
  }
}
