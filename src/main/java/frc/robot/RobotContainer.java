// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.driverCommand.DriveCommands;
import frc.robot.commands.driverCommand.HighAlgaeCommand;
import frc.robot.commands.driverCommand.L1Command;
import frc.robot.commands.driverCommand.L2Command;
import frc.robot.commands.driverCommand.L3Command;
import frc.robot.commands.driverCommand.L4Command;
import frc.robot.commands.driverCommand.LowAlgaeCommand;
import frc.robot.commands.driverCommand.ProcessorCommand;
import frc.robot.commands.driverCommand.StationCommand;
import frc.robot.commands.elevatorCommand.ElevatorDefaultIdleCommand;
import frc.robot.commands.elevatorCommand.ElevatorHomeCommand;
import frc.robot.commands.operatorCommand.ElevatorManualDownCommand;
import frc.robot.commands.operatorCommand.ElevatorManualUpCommand;
import frc.robot.commands.operatorCommand.FreezeCommand;
import frc.robot.commands.operatorCommand.PivotManualForwardCommand;
import frc.robot.commands.operatorCommand.PivotManualReverseCommand;
import frc.robot.commands.operatorCommand.RollerManualForwardCommand;
import frc.robot.commands.operatorCommand.RollerManualReverseCommand;
import frc.robot.commands.pivotCommand.PivotDefaultIdleCommand;
import frc.robot.commands.rollerCommand.RollerDefaultIdleCommand;
import frc.robot.generated.JoystickConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive =
      new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));
  ;
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

  private SendableChooser<Command> autoChooser;
  private SendableChooser<Command> pipelineList;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(JoystickConstants.driverControllerPort);

  private final CommandXboxController testController =
      new CommandXboxController(JoystickConstants.testControllerPort);

  private final CommandXboxController SysIDController =
      new CommandXboxController(JoystickConstants.sysIDControllerPort);

  private final CommandXboxController operatorController =
      new CommandXboxController(JoystickConstants.operatorControllerPort);

  // driver triggers
  private final Trigger DriverL1Trigger =
      driverController.a().and(driverController.rightBumper().negate());
  private final Trigger DriverL2Trigger =
      driverController.b().and(driverController.rightBumper().negate());
  private final Trigger DriverL3Trigger =
      driverController.x().and(driverController.rightBumper().negate());
  private final Trigger DriverL4Trigger = driverController.y();
  private final Trigger DriverStationTrigger = driverController.leftTrigger();
  private final Trigger DriverLowAlgaeTrigger =
      driverController.a().and(driverController.rightBumper());
  private final Trigger DriverHighAlgaeTrigger =
      driverController.b().and(driverController.rightBumper());
  private final Trigger DriverProcessorTrigger =
      driverController.x().and(driverController.rightBumper());

  // operator triggers
  private final Trigger OperatorSwitchTrigger = operatorController.leftBumper();
  private final Trigger OperatorHomeTrigger = operatorController.rightBumper();
  private final Trigger OperatorFreezeTrigger = operatorController.leftTrigger();
  private final Trigger OperatorElevatorUpTrigger =
      new Trigger(() -> operatorController.getLeftX() > 0.4);
  private final Trigger OperatorElevatorDownTrigger =
      new Trigger(() -> operatorController.getLeftX() < -0.4);
  private final Trigger OperatorPivotForwardTrigger =
      new Trigger(() -> operatorController.getRightX() > 0.4);
  private final Trigger OperatorPivotReverseTrigger =
      new Trigger(() -> operatorController.getRightX() < -0.4);
  private final Trigger OperatorRollerForwardTrigger = operatorController.x();
  private final Trigger OperatorRollerReverseTrigger = operatorController.y();

  // driver commands
  private final Command DriverL1Command =
      new L1Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverL2Command =
      new L2Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverL3Command =
      new L3Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverL4Command =
      new L4Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverLowAlgaeCommand =
      new LowAlgaeCommand(m_rollerSubsystem, m_pivotSubsystem, m_elevatorSubsystem);
  private final Command DriverHighAlgaeCommand =
      new HighAlgaeCommand(m_rollerSubsystem, m_pivotSubsystem, m_elevatorSubsystem);
  private final Command DriverProcessorCommand =
      new ProcessorCommand(m_rollerSubsystem, m_pivotSubsystem, m_elevatorSubsystem);
  private final Command DriverStationCommand = new StationCommand(m_rollerSubsystem);

  // operator commands
  private final Command OperatorHomeCommand = new ElevatorHomeCommand(m_elevatorSubsystem);
  private final Command operatorFreezeCommand =
      new FreezeCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command OperatorElevatorUpCommand =
      new ElevatorManualUpCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command OperatorElevatorDownCommand =
      new ElevatorManualDownCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command OperatorPivotForwardCommand =
      new PivotManualForwardCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command OperatorPivotReverseCommand =
      new PivotManualReverseCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command OperatorRollerForwardCommand =
      new RollerManualForwardCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command OperatorRollerReverseCommand =
      new RollerManualReverseCommand(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);

  // default commandfs
  private final Command ElevatorDefaultIdleCommand =
      new ElevatorDefaultIdleCommand(m_elevatorSubsystem);
  private final Command RollerDefaultIdleCommand = new RollerDefaultIdleCommand(m_rollerSubsystem);
  private final Command PivotDefaultIdleCommand = new PivotDefaultIdleCommand(m_pivotSubsystem);
  private final Command rotate2Apriltagleft = new rotate2Apriltag(drive, "left", driverController);
  private final Command rotate2Apriltagright =
      new rotate2Apriltag(drive, "right", driverController);

  // private final Command rotate2Apriltagleft=
  // new rotate2Apriltag(drive,"left");
  // private final Command rotate2Apriltagright=
  // new rotate2Apriltag(drive, "right");
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Real robot, instantiate hardware IO implementations
    NamedCommands.registerCommand(
        "L2 Command",
        new L2Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem).withTimeout(2.0));
    NamedCommands.registerCommand(
        "IntakeCommand", new StationCommand(m_rollerSubsystem).withTimeout(3.0));
    NamedCommands.registerCommand(
        "L4 Command",
        new L4Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem).withTimeout(5.0));

    DashboardDisplay.layout(autoChooser, pipelineList, drive);

    // Configure the trigger bindings

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    DriverL1Trigger.whileTrue(DriverL1Command);
    DriverL1Trigger.whileTrue(DriverL1Command);
    DriverL2Trigger.whileTrue(DriverL2Command);
    DriverL3Trigger.whileTrue(DriverL3Command);
    DriverL4Trigger.whileTrue(DriverL4Command);
    DriverStationTrigger.whileTrue(DriverStationCommand);
    DriverHighAlgaeTrigger.whileTrue(DriverHighAlgaeCommand);
    DriverLowAlgaeTrigger.whileTrue(DriverLowAlgaeCommand);
    DriverProcessorTrigger.whileTrue(DriverProcessorCommand);

    OperatorHomeTrigger.whileTrue(OperatorHomeCommand);
    OperatorSwitchTrigger.onTrue(
        Commands.runOnce(() -> m_pivotSubsystem.switchHoldAlgae(), m_pivotSubsystem)
            .alongWith(
                Commands.runOnce(() -> m_rollerSubsystem.switchHoldAlgae(), m_rollerSubsystem)));
    OperatorFreezeTrigger.whileTrue(operatorFreezeCommand);
    OperatorElevatorUpTrigger.whileTrue(OperatorElevatorUpCommand);
    OperatorElevatorDownTrigger.whileTrue(OperatorElevatorDownCommand);
    OperatorPivotForwardTrigger.whileTrue(OperatorPivotForwardCommand);
    OperatorPivotReverseTrigger.whileTrue(OperatorPivotReverseCommand);
    OperatorRollerForwardTrigger.whileTrue(OperatorRollerForwardCommand);
    OperatorRollerReverseTrigger.whileTrue(OperatorRollerReverseCommand);

    SysIDController.a()
        .whileTrue(m_pivotSubsystem.PivotTestDynamic(SysIdRoutine.Direction.kForward));
    SysIDController.b()
        .whileTrue(m_pivotSubsystem.PivotTestDynamic(SysIdRoutine.Direction.kReverse));
    SysIDController.x()
        .whileTrue(m_pivotSubsystem.PivotTestQuasistatic(SysIdRoutine.Direction.kForward));
    SysIDController.y()
        .whileTrue(m_pivotSubsystem.PivotTestQuasistatic(SysIdRoutine.Direction.kReverse));

    m_elevatorSubsystem.setDefaultCommand(ElevatorDefaultIdleCommand);
    m_pivotSubsystem.setDefaultCommand(PivotDefaultIdleCommand);
    m_rollerSubsystem.setDefaultCommand(RollerDefaultIdleCommand);
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driverController,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            operatorController.a().getAsBoolean()));

    // Lock to 0° when A button is held
    /*
     * driverController
     * .a()
     * .whileTrue(
     * DriveCommands.joystickDriveAtAngle(
     * drive,
     * () -> -driverController.getLeftY(),
     * () -> -driverController.getLeftX(),
     * () -> new Rotation2d()));
     */

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driverController.povRight().whileTrue(rotate2Apriltagright);
    driverController.povLeft().whileTrue(rotate2Apriltagleft);
    driverController
        .povUp()
        .whileTrue(
            Commands.runEnd(
                () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)),
                () -> drive.stop(),
                drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
