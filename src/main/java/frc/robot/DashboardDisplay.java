package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.PipelineSwitch;
import frc.robot.commands.driverCommand.DriveCommands;
import frc.robot.generated.PipelineIndex;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drive.Drive;

public class DashboardDisplay {
  public static void layout(
      SendableChooser<Command> autoChooser,
      SendableChooser<Command> pipelineList,
      Drive drive,
      PivotSubsystem pivot,
      ElevatorSubsystem elevator,
      RollerSubsystem roller) {
    autoChooser = AutoBuilder.buildAutoChooser("test");
    pipelineList = new SendableChooser<Command>();

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    pipelineList.addOption("ALIGNREEF", new PipelineSwitch(PipelineIndex.ALIGNREEF));
    pipelineList.addOption("ALIGNREEF_LED", new PipelineSwitch(PipelineIndex.ALIGNREEF_LED));
    SmartDashboard.putData("Pipeline Chooser", pipelineList);

    SmartDashboard.putData(drive);
    SmartDashboard.putData(elevator);
    SmartDashboard.putData(pivot);
    SmartDashboard.putData(roller);
  }
}
