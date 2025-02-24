package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

/* Interface encapsulating pivot hardware */
public interface PivotIO {
  /** Updates the set of loggable inputs */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the desired angle of the pivot */
  public default void setAngle(double position) {}

  /** Sets the speed of the pivot to the desired percent output */
  public default void setVoltage(double voltage) {}

  public default void hold() {}

  public default boolean isAtPosition(double position) {
    return false;
  }

  @AutoLog
  public static class PivotIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double acceleration = 0.0;
    public double torqueCurrent = 0.0;
    public double motionMagicTarget = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelcius = 0.0;
    public double s1Position = 0.0;
  }
}
