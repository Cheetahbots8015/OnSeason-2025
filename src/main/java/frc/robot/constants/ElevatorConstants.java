package frc.robot.constants;

public class ElevatorConstants {
  public static final int ELEVATOR_LEADER_ID = 51;
  public static final int ELEVATOR_FOLLOWER_ID = 52;
  public static final int ELEVATOR_HALL_SENSOR_ID = 0;
  public static final String ELEVATOR_CANNAME = "canivore";

  public static final boolean ELEVATOR_NEUTRAL_MODE_COAST = true;
  public static final boolean ELEVATOR_INVERSION = false;
  public static final double ELEVATOR_LEADER_KP = 10.0;
  public static final double ELEVATOR_LEADER_KI = 0.0;
  public static final double ELEVATOR_LEADER_KD = 0.0;
  public static final double ELEVATOR_LEADER_KS = 0.0;
  public static final double ELEVATOR_LEADER_KA = 0.0;
  public static final double ELEVATOR_LEADER_KV = 0.0;
  public static final double ELEVATOR_FOLLOWER_KP = 10.0;
  public static final double ELEVATOR_FOLLOWER_KI = 0.0;
  public static final double ELEVATOR_FOLLOWER_KD = 0.0;
  public static final double ELEVATOR_FOLLOWER_KS = 0.0;
  public static final double ELEVATOR_FOLLOWER_KA = 0.0;
  public static final double ELEVATOR_FOLLOWER_KV = 0.0;
  public static final double ELEVATOR_LEADER_CRUISE_VELOCITY = 50.0;
  public static final double ELEVATOR_LEADER_CRUISE_ACCELERATION = 30.0;
  public static final double ELEVATOR_FOLLOWER_CRUISE_VELOCITY = 50.0;
  public static final double ELEVATOR_FOLLOWER_CRUISE_ACCELERATION = 30.0;
  public static final double ELEVATOR_FORWARD_DUTY_CYCLE_LIMIT = 0.7;
  public static final double ELEVATOR_REVERSE_DUTY_CYCLE_LIMIT = -0.7;
  public static final boolean ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_ENABLE = true;
  public static final double ELEVATOR_LEADER_FORWARD_SOFT_LIMIT_THRESHOLD = 122.0;
  public static final boolean ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_ENABLE = true;
  public static final double ELEVATOR_FOLLOWER_FORWARD_SOFT_LIMIT_THRESHOLD = 122.0;
  public static final double ELEVATOR_FOLLOWER_DIFFERENTIAL_KP = 0.0;
  public static final double ELEVATOR_PEAK_DIFFERENTIAL_VOLTAGE = 0.0;
  public static final double ELEVATOR_SIGNAL_UPDATE_FREQUENCY_HZ = 500.0;

  public static final double ELEVATOR_UP_VOLTAGE = 2.0;
  public static final double ELEVATOR_DOWN_VOLTAGE = 4.0;

  public static final double ELEVATOR_HOLD_VOLTAGE = 0.27;
  public static final double ELEVATOR_POSITION_DEADBAND = 1.2;
  public static final double ELEVATOR_MOTION_MAGICIAN_CLOSE_POSITION_DEADBAND_L3L4 = 20.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_HIGH_UP_VOLTAGE_L3L4 = 5.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_HIGH_DOWN_VOLTAGE_L3L4 = -4.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_LOW_UP_VOLTAGE_L3L4 = 0.1;
  public static final double ELEVATOR_MOTION_MAGICIAN_LOW_DOWN_VOLTAGE_L3L4 = 0.1;

  public static final double ELEVATOR_MOTION_MAGICIAN_CLOSE_POSITION_DEADBAND_L1L2 = 5.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_HIGH_UP_VOLTAGE_L1L2 = 4.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_HIGH_DOWN_VOLTAGE_L1L2 = -4.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_LOW_UP_VOLTAGE_L1L2 = 1.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_LOW_DOWN_VOLTAGE_L1L2 = -1.0;

  public static final double ELEVATOR_HOME_UP_TIME = 0.3;
  public static final double ELEVATOR_HOME_UP_VOLTAGE = 3.0;
  public static final double ELEVATOR_HOME_DOWN_VOLTAGE = -2.0;

  public static final double ELEVATOR_DEFAULT_FALL_VOLTAGE = -7.0;
  public static final double ELEVATOR_DEFAULT_SHUT_DOWN_POSITION = 60.0;

  public static final double ELEVATOR_L1_POSITION = 21.68;
  public static final double ELEVATOR_L2_POSITION = 21.68;
  public static final double ELEVATOR_L3_POSITION = 63.0;
  public static final double ELEVATOR_L4_POSITION = 120.0;
  public static final double ELEVATOR_LOW_ALGAE_POSITION = 40.0;
  public static final double ELEVATOR_HIGH_ALGAE_POSITION = 80.0;
  public static final double ELEVATOR_PROCESSOR_POSITION = 0.0;
  public static final double ELEVATOR_STATION_POSITION = 0.0;
}
