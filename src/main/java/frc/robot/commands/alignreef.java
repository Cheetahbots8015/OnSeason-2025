// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;

public class alignreef extends Command {
  private final Drive m_drive;
  private final CommandXboxController m_controller;
  private PIDController pidy;
  private AnalogPotentiometer m_leftrangeFinder;
  private AnalogPotentiometer m_rightrangeFinder;
  private double rangelimit;
  private MedianFilter filter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public alignreef(Drive drive, CommandXboxController controller) {
    m_drive = drive;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-reef", 1);
    LimelightHelpers.setCameraPose_RobotSpace("limelight-reef", 0, -0.25, 0.82, 0, -30, 0);
    LimelightHelpers.setLEDMode_ForceOn("limelight-reef");
    pidy = new PIDController(0.07, 0, 0);
    m_leftrangeFinder = new AnalogPotentiometer(0, 350, 2);
    m_rightrangeFinder = new AnalogPotentiometer(1, 350, 2);
    rangelimit = 35.0; // in centimeters
    filter = new MedianFilter(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-reef");
    if (hasTarget
    // && m_leftrangeFinder.get() < rangelimit
    // && m_rightrangeFinder.get() < rangelimit
    ) {
      if (Math.abs(LimelightHelpers.getTX("limelight-reef")) < 1) {
        m_controller.setRumble(RumbleType.kBothRumble, 0.5);
      } else {
        m_controller.setRumble(RumbleType.kBothRumble, 0);
      }
      SmartDashboard.putBoolean("limelight-reef", false);
      ChassisSpeeds drivSpeeds =
          new ChassisSpeeds(
              0, pidy.calculate(filter.calculate(LimelightHelpers.getTX("limelight-reef"))), 0);
      m_drive.runVelocity(drivSpeeds);
    } else {
      ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, 0, 0);
      m_drive.runVelocity(drivSpeeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(LimelightHelpers.getTX("limelight-reef")) < 0.8;
  }
}
