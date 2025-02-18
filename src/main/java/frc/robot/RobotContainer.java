// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.commands.elevatorCommand.ElevatorDefaultDownCommand;
import frc.robot.commands.elevatorCommand.ElevatorHoldCommand;
import frc.robot.commands.elevatorCommand.ElevatorHomeCommand;
import frc.robot.commands.elevatorCommand.ElevatorL2Command;
import frc.robot.commands.elevatorCommand.ElevatorReportCommand;
import frc.robot.commands.elevatorCommand.ElevatorResetCommand;
import frc.robot.commands.elevatorCommand.ElevatorVoltageOutCommand;
import frc.robot.commands.pivotCommand.PivotDefaultBackCommand;
import frc.robot.commands.pivotCommand.PivotForwardCommand;
import frc.robot.commands.pivotCommand.PivotL2Command;
import frc.robot.commands.pivotCommand.PivotReportCommand;
import frc.robot.commands.pivotCommand.PivotReverseCommand;
import frc.robot.commands.rollerCommand.RollerManualForwardCommand;
import frc.robot.commands.rollerCommand.RollerManualReverseCommand;
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
  private final Drive drive;
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(JoystickConstants.driverControllerPort);

  private final CommandXboxController testController =
      new CommandXboxController(JoystickConstants.testControllerPort);

  private final CommandXboxController SysIDController =
      new CommandXboxController(JoystickConstants.sysIDControllerPort);

  // private final Trigger DriverL1Trigger = driverController.a();
  private final Trigger DriverL2Trigger = driverController.b();
  private final Trigger DriverL3Trigger = driverController.x();
  private final Trigger DriverL4Trigger = driverController.y();
  private final Trigger DriverHomeTrigger = driverController.rightBumper();
  private final Trigger DriverStationTrigger = driverController.leftTrigger();
  private final Trigger DriveerReverseTrigger = driverController.rightTrigger();

  private final Trigger ElevatorManualTrigger = testController.a();
  private final Trigger ElevatorVoltageLockTrigger = testController.b();
  private final Trigger L4Trigger = testController.x();
  private final Trigger L2Trigger = testController.y();
  private final Trigger L3Trigger = testController.povRight();
  private final Trigger RollerManualTrigger = testController.leftBumper();
  private final Trigger RollerReverseTrigger = testController.rightBumper();
  private final Trigger PivotManualForwardTrigger = testController.rightTrigger();
  private final Trigger PivotManualReverseTrigger = testController.leftTrigger();
  private final Trigger PivotL2Trigger = testController.povUp();
  private final Trigger ElevatorHomeTrigger = testController.povDown();

  private final Command DriverL1Command =
      new L1Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverL2Command =
      new L2Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverL3Command =
      new L3Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverL4Command =
      new L4Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command DriverHomeCommand =
      new ElevatorHomeCommand(m_elevatorSubsystem, m_pivotSubsystem);
  private final Command DriverStationCommand = new StationCommand(m_rollerSubsystem);
  private final Command DriverReverseCommand = new RollerManualReverseCommand(m_rollerSubsystem);

  private final Command ElevatorManualCommand = new ElevatorVoltageOutCommand(m_elevatorSubsystem);
  private final Command ElevatorHoldCommand = new ElevatorHoldCommand(m_elevatorSubsystem);
  private final Command ElevatorDefaultDownCommand =
      new ElevatorDefaultDownCommand(m_elevatorSubsystem);
  private final Command RollerManualCommand = new RollerManualForwardCommand(m_rollerSubsystem);
  private final Command RollerReverseCommand = new RollerManualReverseCommand(m_rollerSubsystem);
  private final Command ElevatorReportCommand = new ElevatorReportCommand(m_elevatorSubsystem);
  private final Command ElevatorResetCommand = new ElevatorResetCommand(m_elevatorSubsystem);
  private final Command ElevatorL2Command = new ElevatorL2Command(m_elevatorSubsystem);
  private final Command PivotDefaultBackCommand = new PivotDefaultBackCommand(m_pivotSubsystem);
  private final Command PivotReportCommand = new PivotReportCommand(m_pivotSubsystem);
  private final Command PivotForwardCommand = new PivotForwardCommand(m_pivotSubsystem);
  private final Command PivotReverseCommand = new PivotReverseCommand(m_pivotSubsystem);
  private final Command PivotL2Command = new PivotL2Command(m_pivotSubsystem);
  private final Command ElevatorHomeCommand =
      new ElevatorHomeCommand(m_elevatorSubsystem, m_pivotSubsystem);
  private final Command L2Command =
      new L2Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command L4Command =
      new L4Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);
  private final Command L3Command =
      new L3Command(m_elevatorSubsystem, m_pivotSubsystem, m_rollerSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Real robot, instantiate hardware IO implementations
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
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

    // DriverL1Trigger.whileTrue(DriverL1Command);
    DriverL2Trigger.whileTrue(DriverL2Command);
    DriverL3Trigger.whileTrue(DriverL3Command);
    DriverL4Trigger.whileTrue(DriverL4Command);
    DriverHomeTrigger.whileTrue(DriverHomeCommand);
    DriverStationTrigger.whileTrue(DriverStationCommand);
    DriveerReverseTrigger.whileTrue(DriverReverseCommand);

    ElevatorManualTrigger.whileTrue(ElevatorManualCommand);
    ElevatorVoltageLockTrigger.whileTrue(ElevatorHoldCommand);
    L4Trigger.whileTrue(L4Command);
    L2Trigger.whileTrue(L2Command);
    L3Trigger.whileTrue(L3Command);
    RollerManualTrigger.whileTrue(RollerManualCommand);
    RollerReverseTrigger.whileTrue(RollerReverseCommand);
    m_elevatorSubsystem.setDefaultCommand(ElevatorDefaultDownCommand);
    m_pivotSubsystem.setDefaultCommand(PivotDefaultBackCommand);
    PivotManualForwardTrigger.whileTrue(PivotForwardCommand);
    PivotManualReverseTrigger.whileTrue(PivotReverseCommand);
    ElevatorHomeTrigger.whileTrue(ElevatorHomeCommand);
    PivotL2Trigger.whileTrue(PivotL2Command);

    SysIDController.a()
        .whileTrue(m_pivotSubsystem.PivotTestDynamic(SysIdRoutine.Direction.kForward));
    SysIDController.b()
        .whileTrue(m_pivotSubsystem.PivotTestDynamic(SysIdRoutine.Direction.kReverse));
    SysIDController.x()
        .whileTrue(m_pivotSubsystem.PivotTestQuasistatic(SysIdRoutine.Direction.kForward));
    SysIDController.y()
        .whileTrue(m_pivotSubsystem.PivotTestQuasistatic(SysIdRoutine.Direction.kReverse));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    /*
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));
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
    driverController.povRight().whileTrue(DriveCommands.rotate2Apriltagright(drive));
    driverController.povLeft().whileTrue(DriveCommands.rotate2Apriltagleft(drive));
    driverController
        .pov(0)
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
    return null;
  }
}
