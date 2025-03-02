// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommand;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.ElevatorConstants;
import frc.robot.generated.JoystickConstants;
import frc.robot.generated.PivotConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class L2Command extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevatorSubsystem;

  private final PivotSubsystem m_pivotSubsystem;
  private final XboxController m_controller =
      new XboxController(JoystickConstants.operatorControllerPort);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public L2Command(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_pivotSubsystem = pivotSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevatorSubsystem, m_pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevatorSubsystem.setSystemIdleState(elevatorIdleState.coral);
    // m_pivotSubsystem.setSystemIdleState(pivotIdleState.coral);
    // m_rollerSubsystem.setSystemIdleState(rollerIdleState.coral);
    m_elevatorSubsystem.resetFilter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.L2();
    m_pivotSubsystem.L2();
    if (m_elevatorSubsystem.isAtPosition(ElevatorConstants.L2Position)
        && m_pivotSubsystem.isAtPosition(PivotConstants.L2Position)) {
      m_controller.setRumble(RumbleType.kBothRumble, 0.3);
    } else {
      m_controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_elevatorSubsystem.setSystemIdleState(elevatorIdleState.coral);
    // m_pivotSubsystem.setSystemIdleState(pivotIdleState.coral);
    // m_rollerSubsystem.setSystemIdleState(rollerIdleState.coral);
    m_elevatorSubsystem.defaultDown();
    m_pivotSubsystem.idle();
    m_controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
