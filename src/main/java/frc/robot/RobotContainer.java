// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Generated.JoystickConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorL2Command;
import frc.robot.commands.ElevatorReportCommand;
import frc.robot.commands.ElevatorResetCommand;
import frc.robot.commands.ElevatorVoltageLockCommand;
import frc.robot.commands.ElevatorVoltageOutCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RollerVoltageOutCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RollerSubsystem;

import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(JoystickConstants.driverControllerPort);

  private final CommandXboxController testController =
      new CommandXboxController(JoystickConstants.testControllerPort);

  private final Trigger ElevatorManualTrigger = testController.a();
  private final Trigger ElevatorVoltageLockTrigger = testController.b();
  private final Trigger ElevatorResetTrigger = testController.x();
  private final Trigger ElevatorL2Trigger = testController.y();
  private final Trigger RollerManualTrigger = testController.leftBumper();
  private final Command ElevatorManualCommand = new ElevatorVoltageOutCommand(m_elevatorSubsystem);
  private final Command ElevatorVoltageLockCommand = new ElevatorVoltageLockCommand(m_elevatorSubsystem);
  private final Command RollerManualCommand = new RollerVoltageOutCommand(m_rollerSubsystem);
  private final Command ElevatorReportCommand = new ElevatorReportCommand(m_elevatorSubsystem);
  private final Command ElevatorResetCommand = new ElevatorResetCommand(m_elevatorSubsystem);
  private final Command ElevatorL2Command = new ElevatorL2Command(m_elevatorSubsystem);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    ElevatorManualTrigger.whileTrue(ElevatorManualCommand);
    ElevatorVoltageLockTrigger.whileTrue(ElevatorVoltageLockCommand);
    ElevatorResetTrigger.whileTrue(ElevatorResetCommand);
    ElevatorL2Trigger.whileTrue(ElevatorL2Command);
    RollerManualTrigger.whileTrue(RollerManualCommand);
    m_elevatorSubsystem.setDefaultCommand(ElevatorReportCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
