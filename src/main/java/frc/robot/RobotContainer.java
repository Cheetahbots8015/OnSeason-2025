// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.driverCommand.*;
import frc.robot.commands.elevatorCommand.ElevatorDefaultIdleCommand;
import frc.robot.commands.elevatorCommand.ElevatorHomeCommand;
import frc.robot.commands.operatorCommand.*;
import frc.robot.commands.pivotCommand.PivotDefaultIdleCommand;
import frc.robot.commands.rollerCommand.RollerDefaultIdleCommand;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSystemIOKrakenX60;
import frc.robot.subsystems.pivot.PivotIOKrakenX60;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.rollers.RollerSubsystem;
import frc.robot.subsystems.rollers.RollerSystemIOKrakenX60;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final ElevatorSubsystem m_elevatorSubsystem =
      new ElevatorSubsystem("Elevator", new ElevatorSystemIOKrakenX60() {});
  private final RollerSubsystem m_rollerSubsystem =
      new RollerSubsystem("Roller", new RollerSystemIOKrakenX60());
  private final PivotSubsystem m_pivotSubsystem =
      new PivotSubsystem("Pivot", new PivotIOKrakenX60());

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

  private final Command ElevatorDefaultIdleCommand =
      new ElevatorDefaultIdleCommand(m_elevatorSubsystem);
  private final Command RollerDefaultIdleCommand = new RollerDefaultIdleCommand(m_rollerSubsystem);
  private final Command PivotDefaultIdleCommand = new PivotDefaultIdleCommand(m_pivotSubsystem);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(cameraReefName, drive::getRotation),
                new VisionIOLimelight(cameraStationName, drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(cameraReefName, robotToCameraReef, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraStationName, robotToCameraStation, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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
        frc.robot.commands.driverCommand.DriveCommands.joystickDrive(
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
    return autoChooser.get();
  }
}
