// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.RollerConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  private final CANrange canRange = new CANrange(RollerConstants.ROLLER_CANRANGE_ID, "dabus");
  private final CANrangeConfiguration configs;

  private final SysIdRoutine m_SysIdRoutineRoller;
  private final SysIdRoutine routineToApply;

  private RollerState systemState = RollerState.INITIALIZE;

  private boolean requestManual = false;
  private double manualVoltage = 0.0;
  private boolean requestVelocity = false;
  private String velocityString = null;

  private enum RollerState {
    IDLE,
    LOAD,
    SHOOT,
    INVERT,
    MANUAL,
  }

  public RollerState getSystemState() {
    return systemState;
  }

  public RollerSystem(String name, RollerSystemIO io) {
    configs = new CANrangeConfiguration();
    canRange.getConfigurator().apply(configs);
    this.name = name;
    this.io = io;
    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    m_SysIdRoutineRoller =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2.4),
                Seconds.of(3),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  io.runVolts(volts.magnitude());
                },
                null,
                this));

    routineToApply = m_SysIdRoutineRoller;
  }

  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs(name, inputs);
    Logger.recordOutput(name, this.getSystemState());
    disconnected.set(!inputs.connected);
    updateStateMachine();
  }

  private void updateStateMachine() {
   
  }

  private boolean hasCoral() {
    return canRange.getDistance().getValueAsDouble() < RollerConstants.CANRANGE_DISRANCE_THRESHOLD;
  }

  private boolean hasAlgea() {
    // need to fill in with actual content
    return false;
  }

  @AutoLogOutput
  public Command runRoller(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
  }

  public Command RollerTestDynamic(SysIdRoutine.Direction direction) {
    return routineToApply.dynamic(direction);
  }

  public Command RollerTestQuasistatic(SysIdRoutine.Direction direction) {
    return routineToApply.quasistatic(direction);
  }

  public void setRequestManual(boolean set) {
    requestManual = set;
  }

  public void setManualVoltage(double value) {
    manualVoltage = value;
  }

  public void setRequestVelocity(boolean set) {
    requestVelocity = set;
  }

  public void setVelocityString(String value) {
    velocityString = value;
  }
}
