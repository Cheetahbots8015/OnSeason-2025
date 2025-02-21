package frc.robot.constants;

public class ElevatorConstants {
  public static final int ELEVATOR_LEFT_ID = 51;
  public static final int ELEVATOR_RIGHT_ID = 52;
  public static final int ELEVATOR_HALL_SENSOR_ID = 0;
  public static final String ELEVATOR_CANNAME = "canivore";

  public static final boolean ELEVATOR_INVERSION = false;
  public static final double ELEVATOR_FOLLOWER_KP = 2.0;
  public static final double ELEVATOR_OPEN_LOOP_RAMP_PERIOD = 0.9;
  public static final double ELEVATOR_PEAK_FORWARD_DUTY_CYCLE = 0.6;
  public static final double ELEVATOR_PEAK_REVERSE_DUTY_CYCLE = -0.6;
  public static final double ELEVATOR_PEAK_FOLLOWER_FORWARD_DUTY_CYCLE = 0.8;
  public static final double ELEVATOR_PEAK_FOLLOWER_REVERSE_DUTY_CYCLE = -0.8;
  public static final double ELEVATOR_MOTION_MAGIC_ACCELERATION = 30.0;
  public static final double ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY = 50.0;
  public static final double ELEVATOR_PEAK_DIFFERENTIAL_VOLTAGE = 4.0;

  public static final double ELEVATOR_MOTION_MAGICIAN_UP_RATE = 10.0 / 40.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_UP_MAXIMUM = 10.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_DOWN_RATE = -6.0 / 40.0;
  public static final double ELEVATOR_MOTION_MAGICIAN_DOWN_MAXIMUM = -6.0;

  public static final double SIGNAL_UPDATE_FREQUENCY_HZ = 500.0;
  public static final double CONTROL_UPDATE_FREQUENCY_HZ = 500.0;

  public static final double ELEVATOR_UP_VOLTAGE = 6.0;
  public static final double ELEVATOR_DOWN_VOLTAGE = -6.0;
  public static final double ELEVATOR_LOW_UP_VOLTAGE = 1.0;
  public static final double ELEVATOR_LOW_DOWN_VOLTAGE = -0.8;
  public static final double ELEVATOR_HOLD_VOLTAGE = 0.3;
  public static final double ELEVATOR_POSITION_DEADBAND = 0.5;
  public static final double ELEVATOR_CLOSE_POSITION_DEADBAND = 5.0;

  public static final double ELEVATOR_IDLE_POSITION = 0.5;
  public static final double ELEVATOR_L1_POSITION = 21.68;
  public static final double ELEVATOR_L2_POSITION = 21.68;
  public static final double ELEVATOR_L3_POSITION = 63.0;
  public static final double ELEVATOR_L4_POSITION = 120.0;
  public static final double ELEVATOR_LOW_ALGAE_POSITION = 40.0;
  public static final double ELEVATOR_HIGH_ALGAE_POSITION = 80.0;
  public static final double ELEVATOR_PROCESSOR_POSITION = 0.0;
  public static final double ELEVATOR_STATION_POSITION = 0.0;
  public static final double ELEVATOR_LOLIPOP_POSITION = 0.0;
  public static final double ELEVATOR_HOME_UP_TIME = 0.1;
}
