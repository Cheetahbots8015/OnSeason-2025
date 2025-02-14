package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class VisionCommands {

  public VisionCommands() {}

  public static Command aimAtLeftReef(Drive drive, Vision vision) {
    double[] speed = vision.calculateChassisInput(0, true);
    return Commands.run(
        () -> {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  speed[0], speed[1], speed[2], drive.getRotation()));
        },
        drive,
        vision);
  }

  public static Command aimAtRightReef(Drive drive, Vision vision) {
    double[] speed = vision.calculateChassisInput(0, false);
    return Commands.run(
        () -> {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  speed[0], speed[1], speed[2], drive.getRotation()));
        },
        drive,
        vision);
  }

  public static Command aimAtStation(Drive drive, Vision vision) {
    double[] speed = vision.calculateChassisInput(1, true);
    return Commands.run(
        () -> {
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  speed[0], speed[1], speed[2], drive.getRotation()));
        },
        drive,
        vision);
  }
}
