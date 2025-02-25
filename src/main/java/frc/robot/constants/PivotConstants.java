package frc.robot.constants;

public class PivotConstants {
  public static final int PIVOT_ID = 61;
  public static final int CANDI_ID = 6;
  public static final String PIVOT_CANNAME = "canivore";

  public static final boolean PIVOT_INVERSION = true;
  public static final boolean PIVOT_NEUTRAL_MODE_COAST = false;
  public static final double PIVOT_KP = 68.3;
  public static final double PIVOT_KI = 0.0;
  public static final double PIVOT_KD = 0.0;
  public static final double PIVOT_KA = 0.0;
  public static final double PIVOT_KS = 0.3;
  public static final double PIVOT_KV = 0.5;
  public static final double PIVOT_FORWARD_DUTY_CYCLE_LIMIT = 0.55;
  public static final double PIVOT_REVERSE_DUTY_CYCLE_LIMIT = -0.55;
  public static final boolean PIVOT_FORWARD_SOFT_LIMIT_ENABLE = true;
  public static final boolean PIVOT_REVERSE_SOFT_LIMIT_ENABLE = true;
  public static final double PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD = 0.45;
  public static final double PIVOT_REVERSE_SOFT_LIMIT_THRESHOLD = 0.0;
  public static final double PIVOT_CANDI_2_MECHANISM_RATIO = 1.0;
  public static final double PIVOT_ROTOR_2_CANDI_RATIO = 37.33333;
  public static final double PIVOT_POSITION_TOLERANCE = 0.03;
  public static final int PIVOT_SIGNAL_UPDATE_FREQUENCY_HZ = 500;

  public static final boolean CANDI_DIRECTION = false;
  public static final double CANDI_OFFSET = -0.7;

  public static final double PIVOT_MOTION_MAGIC_ACCELERATION = 1.7;
  public static final double PIVOT_MOTION_MAGIC_CRUISE_VELOCITY = 1.07;
  public static final double PIVOT_MANUAL_FORWARD_VOLTAGE = 0.5;
  public static final double PIVOT_MANUAL_REVERSE_VOLTAGE = -0.5;

  public static final double PIVOT_L1_ANGLE = 0.0375;
  public static final double PIVOT_L2_ANGLE = 0.0375;
  public static final double PIVOT_L3_ANGLE = 0.0375;
  public static final double PIVOT_L4_ANGLE = 3.5 / PIVOT_ROTOR_2_CANDI_RATIO; // TODO: change
  public static final double PIVOT_HIGHALGAE_ANGLE = 0.45;
  public static final double PIVOT_LOWALGAE_ANGLE = 0.45;
  public static final double PIVOT_PROCESSOR_ANGLE = 0.0;
  public static final double PIVOT_ALGAE_HOLD_ANGLE = 0.0; //TODO: need to chagne
  public static final double PIVOT_CORAL_HOLD_ANGLE= 0.0;
}
