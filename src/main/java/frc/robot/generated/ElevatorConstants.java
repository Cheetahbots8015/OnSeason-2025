package frc.robot.generated;

public class ElevatorConstants {
  public static final int ELEVATOR_LEFT_ID = 52;
  public static final int ELEVATOR_RIGHT_ID = 51;
  public static final int ELEVATOR_HALL_SENSOR_ID = 0;

  public static final boolean ELEVATOR_LEFT_INVERSION = false;
  public static final boolean ELEVATOR_LEFT_BRAKE = true;
  public static final boolean ELEVATOR_RIGHT_BRAKE = true;
  public static final boolean OPPOSE_MASTER = true;

  public static final boolean ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_ENABLE = false;
  public static final boolean ELEVATOR_LEADER_REVERSE_SOFT_LIMIT_ENABLE = false;
  public static final boolean ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_ENABLE = false;
  public static final boolean ELEVATOR_FOLLOWER_REVERSE_SOFT_LIMIT_ENABLE = false;
  public static final double ELEVATOR_LEADER_SOFT_LIMIT_FORWARD = 200.0;
  public static final double ELEVATOR_LEADER_SOFT_LIMIT_REVERSE = -200.0;
  public static final double ELEVATOR_FOLLOWER_SOFT_LIMIT_FORWARD = 200.0;
  public static final double ELEVATOR_FOLLOWER_SOFT_LIMIT_REVERSE = -200.0;

  public static final boolean ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE = true;
  public static final boolean ELEVATOR_SUPPLY_CURRENT_LIMIT_ENABLE = true;
  public static final double ELEVATOR_STATOR_CURRENT_LIMIT_AMPS = 100.0;
  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
  public static final double ELEVATOR_SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 40.0;
  public static final double ELEVATOR_SUPPLY_CURRENT_LOWER_TIME = 1.0;

  public static final double ELEVATOR_PEAK_FORWARD_DUTY_CYCLE = 0.7;
  public static final double ELEVATOR_PEAK_REVERSE_DUTY_CYCLE = -0.7;

  public static final double ELEVATOR_DUTYCYCLE_CLOSEDLOOP_RAMP_PERIOD = 0.5;
  public static final double ELEVATOR_TORQUE_CLOSEDLOOP_RAMP_PERIOD = 0.5;
  public static final double ELEVATOR_VOLTAGE_CLOSEDLOOP_RAMP_PERIOD = 0.5;

  public static final double ELEVATOR_KP = 0.5;
  public static final double ELEVATOR_KI = 0.0;
  public static final double ELEVATOR_KD = 0.0;
  public static final double ELEVATOR_KA = 0.0;
  public static final double ELEVATOR_KS = 0.0;
  public static final double ELEVATOR_KV = 0.0;
  public static final double ELEVATOR_MOTION_MAGIC_ACCELERATION = 60.0;
  public static final double ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY = 300.0;
  public static final double ELEVATOR_MOTION_MAGIC_JERK = 60.0;

  public static final double SIGNAL_UPDATE_FREQUENCY_HZ = 50.0;
  public static final double CONTROL_UPDATE_FREQUENCY_HZ = 50.0;

  public static final double ELEVATOR_HOME_UP_TIME = 5.0;
  public static final double ELEVATOR_HOME_UP_VOLTAGE = 0.3;
  public static final double ELEVATOR_HOME_DOWN_VOLTAGE = -ELEVATOR_HOME_UP_VOLTAGE;
  public static final double ELEVATOR_HOME_POSITION_TOLERANCE_RADS = 0.5;
  public static final double ELEVATOR_SET_POSITION_TOLERANCE_RADS = 2.5;
  public static final double ELEVATOR_HOME_POSITION_RADS = 0.0;
  public static final double ELEVATOR_L1_POSITION_RADS = 0.0;
  public static final double ELEVATOR_L2_POSITION_RADS = 150.0;
  public static final double ELEVATOR_L3_POSITION_RADS = -70.0;
  public static final double ELEVATOR_L4_POSITION_RADS = 0.0;
  public static final double ELEVATOR_BARGE_POSITION_RADS = 0.0;
  public static final double ELEVATOR_PROCESSOR_POSITION_RADS = 0.0;
  public static final double ELEVATOR_STATION_POSITION_RADS = 0.0;
  public static final double ELEVATOR_LOW_ALGAE_POSITION_RADS = 0.0;
  public static final double ELEVATOR_HIGH_ALGAE_POSITION_RADS = 0.0;
}
