// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.ElevatorConstants;
import frc.robot.generated.PivotConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class L2Command extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevatorSubsystem;

  private final PivotSubsystem m_pivotSubsystem;
  private final RollerSubsystem m_rollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public L2Command(
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
    m_elevatorSubsystem.set2L2();
    m_pivotSubsystem.set2L2();
    SmartDashboard.putBoolean(
        "elevator at pos", m_elevatorSubsystem.isAtPosition(ElevatorConstants.L2Position));
    SmartDashboard.putBoolean(
        "pivot at pos", m_pivotSubsystem.isAtPosition(PivotConstants.L2Position));
    if (m_elevatorSubsystem.isAtPosition(ElevatorConstants.L2Position)
        && m_pivotSubsystem.isAtPosition(PivotConstants.L2Position)) {
      m_rollerSubsystem.L2Vots();
    } else {
      m_rollerSubsystem.defaultIdelVelocity();
    }
    m_elevatorSubsystem.report();
    m_pivotSubsystem.report();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerSubsystem.defaultIdelVelocity();
    m_elevatorSubsystem.shutDown();
    m_pivotSubsystem.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
