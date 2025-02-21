// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String cameraReefName = "camera_Reef";
  public static int cameraReefIndex = 0;
  public static String cameraStationName = "camera_Station";
  public static int cameraStationIndex = 1;

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCameraReef =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCameraStation =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Multipliers for calculation of x, y and rot of the robot
  public static double kPVerticalAim = 1.0; // x, m/s
  public static double kPHorizontalAim = 1.0; // y
  public static double kPRotationalAim = 1.0; // rot

  // threshold distance (in meter) for tracking and reached
  public static double trackingDistance2Reef = 1.0; // meter
  public static double trackingDistance2Station = 1.0;
  public static double reachedDistance2Reef = 1.0;
  public static double reachedDistance2Station = 1.0;

  // parameter of the camera on the robot
  // looking down means negative tilt
  public static double reefCamHeight = 1.0;
  public static double reefHeight = 0.3;
  public static double reefCamTilt = 0.5;
  public static double stationCamHeight = 1.0;
  public static double stationHeight = 1.3;
  public static double stationCamTilt = 0.3;

  public static double reefCamXShift = 0.2;
  public static double reefCamYShift = 0.15;

  public static double reefTagShift = 0.5;
}
