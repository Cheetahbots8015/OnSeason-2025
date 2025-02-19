// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;

public class rotate2Apriltag extends Command {
  private final Drive m_drive;
  private final String m_direction;
  PIDController pidx = new PIDController(8, 0, 0);
  PIDController pidy = new PIDController(5, 0, 0);
  PIDController pidyaw = new PIDController(5, 0, 0);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public rotate2Apriltag(Drive drive, String direction) {
    m_drive = drive;
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_direction == "right") {
      LimelightHelpers.setPipelineIndex("limelight-reef", 0);
      LimelightHelpers.SetFidcuial3DOffset("limelight-reef", 0, -0.2, 0);
      LimelightHelpers.setCameraPose_RobotSpace("limelight-reef", 0, -0.25, 0.82, 0, -30, 0);
    } else {
      LimelightHelpers.setPipelineIndex("limelight-reef", 0);
      LimelightHelpers.SetFidcuial3DOffset("limelight-reef", 0, 0.2, 0);
      LimelightHelpers.setCameraPose_RobotSpace("limelight-reef", 0, -0.25, 0.82, 0, -30, 0);
    }
    LimelightHelpers.setLEDMode_ForceOff("limelight-reef");
    pidx.setSetpoint(0.8);
    pidy.setSetpoint(0);
    pidyaw.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-reef");
    Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-reef");
    if (hasTarget) {
      if (pose.getTranslation().getZ() <= 0.8) {
        SmartDashboard.putBoolean("limelight-reef", false);
        pidy = new PIDController(0.07, 0, 0);
        pidyaw = new PIDController(0.05, 0, 0);
        ChassisSpeeds drivSpeeds =
            new ChassisSpeeds(
                0,
                pidy.calculate(LimelightHelpers.getTX("limelight-reef")),
                pidyaw.calculate(pose.getRotation().getY()));
        m_drive.runVelocity(drivSpeeds);
      } else {
        ChassisSpeeds drivSpeeds =
            new ChassisSpeeds(
                -pidx.calculate(pose.getTranslation().getZ()),
                pidy.calculate(pose.getTranslation().getX()),
                pidyaw.calculate(pose.getRotation().getY()));
        m_drive.runVelocity(drivSpeeds);
      }
    } else {
      ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, 0, 0);
      m_drive.runVelocity(drivSpeeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Commands.runEnd(
        () -> m_drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), () -> m_drive.stop(), m_drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-reef").getTranslation().getZ()
            <= 0.8
        && Math.abs(LimelightHelpers.getTX("limelight-reef")) < 1.4;
  }
}
