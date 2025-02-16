// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Generated.ElevatorConstants;
import frc.robot.Generated.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TakeLolipopCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RollerSubsystem m_rollerSubsystem;
  private final PivotSubsystem m_pivotSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TakeLolipopCommand(RollerSubsystem rollerSubsystem, PivotSubsystem pivotSubsystem) {
    this.m_rollerSubsystem = rollerSubsystem;
    this.m_pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollerSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivotSubsystem.set2L2();
    if (m_pivotSubsystem.isAtPosition(PivotConstants.L2Position)) {
        m_rollerSubsystem.LolipopVolts();
    }
    m_pivotSubsystem.report();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivotSubsystem.hold();
    m_rollerSubsystem.shutDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
