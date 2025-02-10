// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorSystemIO {
  @AutoLog
  static class ElevatorSystemIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadsPerSec2 = 0.0;
    public double[] appliedVoltage = new double[] {}; // {leader, follower}
    public double[] motionMagicPositionTarget = new double[] {}; // {leader, follower}
    public double[] supplyCurrentAmps = new double[] {}; // {leader, follower}
    public double[] torqueCurrentAmps = new double[] {}; // {leader, follower}
    public double[] tempCelcius = new double[] {}; // {leader, follower}
  }

  default void updateInputs(ElevatorSystemIOInputs inputs) {}

  /* Updates tunable numbers if neccesary */
  default void updateTunableNumbers() {}

  /* Run elevator at volts */
  default void setVolts(double volts) {}

  /* Run elevator at motionmagic foc */
  default void setHeightRads(double height) {}

  default void setEncoder2Zero() {
  }

  default double getEncoderPositionRads() {
    return -1.0;
  }

  /* Stop elevator */
  default void stop() {}
}
