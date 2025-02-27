// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.LimelightHelpers;
// import frc.robot.generated.PipelineIndex;
// import frc.robot.subsystems.drive.Drive;

// public class PipelineSwitch extends Command {
//   private final PipelineIndex m_index;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public PipelineSwitch(Drive drive, PipelineIndex index) {
//     m_index = index;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(drive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     LimelightHelpers.setPipelineIndex("limelight-reef", m_index.ordinal());
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     int currentIndex = (int) LimelightHelpers.getCurrentPipelineIndex("limelight-reef");
//     return (currentIndex == m_index.ordinal());
//   }
// }
