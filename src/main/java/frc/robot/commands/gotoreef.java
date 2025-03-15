// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;

public class gotoreef extends Command {
  private final Drive m_drive;
  private PIDController pidx;
  private PIDController pidy;
  private PIDController pidyaw;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public gotoreef(Drive drive) {
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-reef", 0);
    pidx = new PIDController(4, 0, 0);
    pidy = new PIDController(2, 0, 0);
    pidyaw = new PIDController(2, 0, 0);
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
      ChassisSpeeds drivSpeeds =
          new ChassisSpeeds(
              -pidx.calculate(pose.getTranslation().getZ()),
              pidy.calculate(pose.getTranslation().getX()),
              pidyaw.calculate(pose.getRotation().getY()));
      m_drive.runVelocity(drivSpeeds);
    } else {
      ChassisSpeeds drivSpeeds = new ChassisSpeeds(0.6, 0, 0);
      m_drive.runVelocity(drivSpeeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
