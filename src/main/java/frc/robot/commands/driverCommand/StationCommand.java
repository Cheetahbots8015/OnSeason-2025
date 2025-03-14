// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PivotSubsystem.pivotIdleState;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.RollerSubsystem.rollerIdleState;

/** An example command that uses an example subsystem. */
public class StationCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RollerSubsystem m_rollerSubsystem;

  private final PivotSubsystem m_pivotSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StationCommand(RollerSubsystem rollerSubsystem, PivotSubsystem pivotSubsystem) {
    m_rollerSubsystem = rollerSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rollerSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rollerSubsystem.setSystemIdleState(rollerIdleState.coral);
    m_pivotSubsystem.setSystemIdleState(pivotIdleState.coral);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollerSubsystem.station();
    m_pivotSubsystem.station();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerSubsystem.defaultIdelVelocity();
    m_pivotSubsystem.idle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
