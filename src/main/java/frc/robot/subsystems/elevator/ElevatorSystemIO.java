// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorSystemIO {
  default void updateInputs(ElevatorSystemIOInputs inputs) {}

  default void setHome() {}

  default void hold() {}

  default void stop() {}

  default void setVoltage(double voltage) {}

  default void defaultFall() {}

  default void setPosition_MotionMagic(double position) {}

  default void setPosition_MotionMagician(double position) {}

  default void setPosition_MotionMagicianLow(double position) {}

  default boolean isAtPosition(double pos) {
    return false;
  }

  default boolean getHomed() {
    return false;
  }

  default void resetFilter() {}

  default void resetHomePhase() {}

  @AutoLog
  static class ElevatorSystemIOInputs {
    public boolean connected = false;
    public double positionDiff = 0.0;
    public double currentDiff = 0.0;
    public double dutyCycleDiff = 0.0;
    public double voltageDiff = 0.0;
    public double[] position = new double[] {};
    public double[] positionWithoutOffset = new double[] {};
    public double[] velocity = new double[] {};
    public double[] acceleration = new double[] {};
    public double[] appliedVoltage = new double[] {}; // {leader, follower}
    public double[] motionMagicPositionTarget = new double[] {}; // {leader, follower}
    public double[] supplyCurrentAmps = new double[] {}; // {leader, follower}
    public double[] torqueCurrentAmps = new double[] {}; // {leader, follower}
    public double[] encoderOffset = new double[] {}; // {leader, follower}
  }
}
