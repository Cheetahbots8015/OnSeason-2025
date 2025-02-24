package frc.robot.generated;

public class PivotConstants {
  // can names and can IDs
  public static final int pivotID = 61;
  public static final int candiID = 6;
  public static final String canName = "canivore";

  // krakenX60 configs
  public static boolean inverted_CounterClockwisePositive = true;
  public static boolean neutalmode_Coast = false;
  public static final double forwardDutyCycleLimit = 0.55;
  public static final double reverseDutyCycleLimit = -0.55;
  public static final boolean forwardSoftLimitEnable = true;
  public static final boolean reverseSoftLimitEnable = true;
  public static final double forwardSoftLimitThreshold = 0.45;
  public static final double reverseSoftLimitThreshold = 0.0;
  public static final double kP = 68.3;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kA = 0.0;
  public static final double kS = 0.3;
  public static final double kV = 0.5;
  public static final double cruiseVelocity = 1.07;
  public static final double cruiseAcceleration = 1.07;

  // candi congis
  public static final boolean candiDirection = false;

  // remote feedback sensor fongis
  public static final double candi2MechanismRatio = 1.0;
  public static final double rotor2CandiRatio = 37.33333;

  // for operator and test
  public static final double manualForwardVoltage = 0.5;
  public static final double manualReverseVoltage = -0.5;

  // home
  public static final double coralHomePosition = 0.002;
  public static final double algaeHomePosition = 0.268;

  // coral reef
  public static final double L1Position = 0.0375;
  public static final double L2Position = 0.0375;
  public static final double L3Position = 0.0375;
  public static final double L4Position = 0.0375;

  // algae
  public static final double lowAlgaePosition = 0.45;
  public static final double highAlgaePosition = 0.45;
  public static final double processorPosition = 0.45;

  // deadband
  public static final double positionDeadband = 0.03;
}
