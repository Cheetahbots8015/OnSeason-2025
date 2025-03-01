package frc.robot.generated;

public class RollerConstants {
  // can names and can IDs
  public static final int rollerID = 62;
  public static final String canName = "canivore";
  public static final int canRangeID = 60;
  // canRange configs
  public static final double canRangeThreshold = 0.06;
  public static final double canRangeHysteresis = 0.005;
  public static final int minSignalStrength = 2500;

  // krankenX60 configs
  public static boolean inverted_CounterClockwisePositive = false;
  public static boolean neutalmode_Coast = true;
  public static final double kP = 10.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kA = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;

  // for operator and test
  public static final double manualForwardVoltage = 8.0;
  public static final double manualReverseVoltage = -2.0;

  // for test
  public static final double manualForwardVelocity = 14.0;
  public static final double manualReverseVelocity = -14.0;

  // for idle
  public static final double algaeIdelVelocity = -25.0;
  public static final double coralIdleVelocity = 0.0;

  // station
  public static final double stationVelocity = 25.0;
  public static final double stationTime = 0.05;

  // coral reef-------------
  public static final double L1Velocity = 6.0;
  public static final double L2Velocity = 45.0;
  public static final double L3Velocity = 45.0;
  public static final double L4Velocity = 45.0;

  // algae
  public static final double intakeAlgaefromReefVelocity = -38.0;
  public static final double processorVelocity = 48.0;
}
