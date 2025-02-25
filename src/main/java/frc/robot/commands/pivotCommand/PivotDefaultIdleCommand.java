package frc.robot.commands.pivotCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.pivot.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class PivotDefaultIdleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PivotDefaultIdleCommand(PivotSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.idle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_subsystem.shutDown();
    m_subsystem.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.getHoldAlgae()) {
      return m_subsystem.isAtPosition(PivotConstants.PIVOT_ALGAE_HOLD_ANGLE);
    } else {
      return m_subsystem.isAtPosition(PivotConstants.PIVOT_CORAL_HOLD_ANGLE);
    }
  }
}
