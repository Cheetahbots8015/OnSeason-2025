package frc.robot.constants;

public class RollerConstants {
  public static final int ROLLER_ID = 62;
  public static final int ROLLER_CANRANGE_ID = 60;
  public static final String ROLLER_CANNAME = "canivore";

  public static final boolean ROLLER_INVERSION = false;
  public static final boolean ROLLER_NEUTRAL_MODE_COAST = true;
  public static final double ROLLER_KP = 10.0;
  public static final double ROLLER_KI = 0.0;
  public static final double ROLLER_KD = 0.0;
  public static final double ROLLER_KA = 0.0;
  public static final double ROLLER_KS = 0.0;
  public static final double ROLLER_KV = 0.0;

  public static final double ROLLER_SIGNAL_UPDATE_FREQUENCY_HZ = 50.0;

  public static final double ROLLER_LOAD_CORAL_VELOCITY = 15.0;
  public static final double ROLLER_LOAD_ALGAE_VELOCITY = -8.0;
  public static final double ROLLER_CORAL_IDLE_VELOCITY = 0.0;
  public static final double ROLLER_ALGAE_IDLE_VELOCITY = -15.0;
  public static final double ROLLER_L1_VELOCITY = 6.0;
  public static final double ROLLER_L2_VELOCITY = 15.0;
  public static final double ROLLER_L3_VELOCITY = 15.0;
  public static final double ROLLER_L4_VELOCITY = 15.0;
  public static final double ROLLER_MANUALFORWARD_VOLTAGE = 8.0;
  public static final double ROLLER_MANUALINVERT_VOLTAGE = -2.0;
  public static final double ROLLER_PROCESSOR_VELOCITY = 8.0;
  public static final double ROLLER_STATION_TIME = 0.15;

  public static final double CANRANGE_DISTANCE_THRESHOLD = 0.06;
  public static final double CANRANGE_MIN_SIGNAL_STRENGTH = 2500;
  public static final double CANRANGE_HYSTERESIS = 0.005;
}
