// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;

public class alignreef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive m_drive;
  private double tagID = -1;
  private boolean aligny;

  public alignreef(boolean isRightScore, Drive drive) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
    this.isRightScore = isRightScore;
    this.m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.aligny = true;
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? 0.1 : -0.22);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-reef");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-reef")
        && LimelightHelpers.getFiducialID("limelight-reef") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-reef");

      double xSpeed = xController.calculate(postions[2]);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);
      if (!yController.atSetpoint() && aligny) {
        ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, ySpeed, rotValue);
        m_drive.runVelocity(drivSpeeds);
      } else {
        yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
        aligny = false;
        ChassisSpeeds drivSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
        m_drive.runVelocity(drivSpeeds);
      }

      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      ChassisSpeeds drivSpeeds = new ChassisSpeeds(0.5, 0, 0);
      m_drive.runVelocity(drivSpeeds);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds drivSpeeds = new ChassisSpeeds(0, 0, 0);
    m_drive.runVelocity(drivSpeeds);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag
    // in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
        || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
