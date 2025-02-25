package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.PipelineIndex;

public class ShuffleboardDisplay extends Command {
  private Shuffleboard m_shuffleboard;

  public ShuffleboardDisplay(Shuffleboard shuffleboard) {
    m_shuffleboard = shuffleboard;
  }

  @Override
  public void execute() {
    SmartDashboard.putData("Choose a pipeline", new PipelineSwitch(PipelineIndex.ALIGNREEF));
  }
}
