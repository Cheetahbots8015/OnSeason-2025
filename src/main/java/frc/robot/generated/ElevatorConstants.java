package frc.robot.generated;

public class ElevatorConstants {
  // can names and can IDs
  public static final int leaderID = 51;
  public static final int followerID = 52;
  public static final int hallSensorID = 0;
  public static final String canName = "canivore";

  // krankenX60 configs
  public static boolean leader_Inverted_CounterClockwisePositive = true;
  public static boolean leader_Neutalmode_Coast = true;
  public static boolean follower_Inverted_CounterClockwisePositive = false;
  public static boolean follower_Neutalmode_Coast = true;
  public static final double leader_kP = 1.6;
  public static final double leader_kI = 0.0;
  public static final double leader_kD = 0.0;
  public static final double leader_kA = 0.0;
  public static final double leader_kS = 0.0;
  public static final double leader_kV = 0.0;
  public static final double follower_kP = 1.6;
  public static final double follower_kI = 0.0;
  public static final double follower_kD = 0.0;
  public static final double follower_kA = 0.0;
  public static final double follower_kS = 0.0;
  public static final double follower_kV = 0.0;
  public static final double forwardDutyCycleLimit = 0.7;
  public static final double reverseDutyCycleLimit = -0.7;
  public static final boolean leader_ForwardSoftLimitEnable = true;
  public static final double leader_forwardSoftLimitThreshold = 122;
  public static final boolean follower_ForwardSoftLimitEnable = true;
  public static final double follower_forwardSoftLimitThreshold = 122;

  // differential configs
  public static final double peakDifferentialVoltage = 4.0;
  public static final double differentialkP = 2.0;

  // motion magic configs
  public static final double leader_CruiseVelocity = 60.0;
  public static final double leader_CruiseAcceleration = 60.0;
  public static final double follower_CruiseVelocity = 60.0;
  public static final double follower_CruiseAcceleration = 60.0;

  // motion magician
  public static final double motionmagicianHighUpVoltage_L3L4 = 6.0;
  public static final double motionmagicianHighDownVoltage_L3L4 = -4.0;
  public static final double motionmagicianLowUpVoltage_L3L4 = 0.1;
  public static final double motionmagicianLowDownVoltage_L3L4 = 0.1;
  public static final double motionmagicianHighUpVoltage_L1L2 = 5.0;
  public static final double motionmagicianHighDownVoltage_L1L2 = -4.0;
  public static final double motionmagicianLowUpVoltage_L1L2 = 1.0;
  public static final double motionmagicianLowDownVoltage_L1L2 = -1.0;

  // set position
  public static final double setPositionUpVoltage = 3.0;
  public static final double setPositionDownvoltage = -2.0;

  // voltages
  public static final double defaultDownVoltage = -5.0;
  public static final double holdVoltage = 0.27;
  public static final double homeUpVoltage = 3.0;
  public static final double homeDownVoltage = -2.0;
  public static final double manualUpVoltage = 3.0;
  public static final double manualDownVoltage = -3.0;

  // positions in rotations
  public static final double homePosition = 0.0;
  public static final double L1Position = 21.68;
  public static final double L2Position = 21.68;
  public static final double L3Position = 59.0;
  public static final double L4Position = 118.0;
  public static final double lowAlgaePosition = 40.0;
  public static final double highAlgaePosition = 80.0;
  public static final double defaultDownShutDownPosition = 60.0;
  public static final double L4PivotFurtherOutPosition = 110.0;

  // time period for moving up while homing the elevator
  public static final double homeUpTime = 0.3;

  // deadband
  public static final double positionDeadband = 1.2;
  public static final double motionmagicianClosePositionDeadband_L3L4 = 22.5;
  public static final double motionmagicianClosePositionDeadband_L1L2 = 5.0;
}
