// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  static class RollerSystemIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double accelerationRadsPerSec2 = 0.0;
    public double TorqueCurrentTarget = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void updateTunableNumbers() {}

  /* Run rollers at volts */
  default void runVolts(double volts) {}

  default void runTorqueCurrentVelocity(double velocity) {}

  /* Stop rollers */
  default void stop() {}
}
