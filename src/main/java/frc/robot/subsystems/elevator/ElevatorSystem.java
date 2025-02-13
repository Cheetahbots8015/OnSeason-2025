// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.ElevatorConstants;
import frc.robot.util.NarcissusUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSystem extends SubsystemBase {
  private final String name;
  private final ElevatorSystemIO io;
  protected final ElevatorSystemIOInputsAutoLogged inputs = new ElevatorSystemIOInputsAutoLogged();
  private final Alert disconnected;
  private final DigitalInput lowLimitSwitch =
      new DigitalInput(ElevatorConstants.ELEVATOR_LOW_LIMITSWITCH_ID);
  private final DigitalInput highLimitSwitch =
      new DigitalInput(ElevatorConstants.ELEVATOR_HIGH_LIMITSWITCH_ID);

  private ElevatorState systemState = ElevatorState.INITIALIZE;

  private boolean requestManual = false;
  private double manualVoltage = 0.0;
  private boolean requestHome = false;
  private boolean requestPosition = false;
  private String positionString = null;

  private boolean homed = false;
  private double homeTimer = -1.0;

  /* System States */
  private enum ElevatorState {
    INITIALIZE,
    HOME,
    L1,
    L2,
    L3,
    L4,
    LOW_ALGAE,
    HIGH_ALGAE,
    PROCESSOR,
    BARGE,
    STATION,
    MANUAL
  }

  private final SysIdRoutine m_SysIdRoutineElevator;

  private final SysIdRoutine routineToApply;

  public ElevatorSystem(String name, ElevatorSystemIO io) {
    this.name = name;
    this.io = io;
    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    m_SysIdRoutineElevator =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2.4),
                Seconds.of(3),
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                  io.setVolts(volts.magnitude());
                },
                null,
                this));

    routineToApply = m_SysIdRoutineElevator;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs(name, inputs);
    Logger.recordOutput(name, this.getSystemState());
    disconnected.set(!inputs.connected);
    updateStateMachine();
  }

  @AutoLogOutput
  public Command runRoller(double inputVolts) {
    return startEnd(() -> io.setVolts(inputVolts), () -> io.stop());
  }

  /* Returns the current system state of the elevator/pivot */
  public ElevatorState getSystemState() {
    return systemState;
  }

  public void updateStateMachine() {

    if (requestManual) {
      systemState = ElevatorState.MANUAL;
      homed = false;
      this.setVoltage(manualVoltage);
      return;
    }

    if (systemState == ElevatorState.INITIALIZE) {
      requestHome = true;
    }

    if (requestHome) {
      this.home();
      if (lowLimitSwitch.get()
          && NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - 0.0,
                  ElevatorConstants.ELEVATOR_HOME_POSITION_TOLERANCE_RADS)
              == 0.0) {
        homed = true;
        systemState = ElevatorState.HOME;
        requestHome = false;
      }
    } else {
      homeTimer = -1.0;
    }

    if (requestPosition && homed) {
      double target = 0.0;
      switch (positionString) {
        case "L1":
          target = ElevatorConstants.ELEVATOR_L1_POSITION_RADS;
          this.set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.L1;
          }
          break;

        case "L2":
          target = ElevatorConstants.ELEVATOR_L2_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.L2;
          }
          break;

        case "L3":
          target = ElevatorConstants.ELEVATOR_L3_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.L3;
          }
          break;

        case "L4":
          target = ElevatorConstants.ELEVATOR_L4_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.L4;
          }
          break;

        case "Barge":
          target = ElevatorConstants.ELEVATOR_BARGE_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.BARGE;
          }
          break;

        case "Processor":
          target = ElevatorConstants.ELEVATOR_PROCESSOR_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.PROCESSOR;
          }
          break;

        case "Station":
          target = ElevatorConstants.ELEVATOR_STATION_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.STATION;
          }
          break;

        case "Low_algae":
          target = ElevatorConstants.ELEVATOR_LOW_ALGAE_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.LOW_ALGAE;
          }
          break;

        case "High_algae":
          target = ElevatorConstants.ELEVATOR_HIGH_ALGAE_POSITION_RADS;
          set2Position(target);
          if (NarcissusUtil.deadband(
                  io.getEncoderPositionRads() - target,
                  ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
              == 0) {
            hold();
            requestPosition = false;
            systemState = ElevatorState.HIGH_ALGAE;
          }
          break;

        default:
          break;
      }
    }
  }

  private void home() {
    if (homeTimer == -1.0) {
      homeTimer = Timer.getFPGATimestamp();
    }

    if (Timer.getFPGATimestamp() - homeTimer < ElevatorConstants.ELEVATOR_HOME_UP_TIME) {
      io.setVolts(ElevatorConstants.ELEVATOR_HOME_UP_VOLTAGE);
    }

    if (Timer.getFPGATimestamp() - homeTimer >= ElevatorConstants.ELEVATOR_HOME_UP_TIME) {
      io.setVolts(ElevatorConstants.ELEVATOR_HOME_DOWN_VOLTAGE);
    }

    if (lowLimitSwitch.get()) {
      io.setEncoder2Zero();
      io.stop();
    }
  }

  private void hold() {
    io.setHeightRads(io.getEncoderPositionRads());
  }

  private void set2Position(double target) {
    io.setHeightRads(target);
    if (NarcissusUtil.deadband(
            io.getEncoderPositionRads() - target,
            ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
        == 0) {
      io.stop();
    }
  }

  private void setVoltage(double voltage) {
    io.setVolts(voltage);
  }

  public void setRequestHome(boolean set) {
    requestHome = set;
  }

  public void setRequestManual(boolean set) {
    requestManual = set;
  }

  public void setRequestPosition(boolean set) {
    requestPosition = set;
  }

  public void setManualVoltage(double value) {
    manualVoltage = value;
  }

  public void setPositionString(String value) {
    positionString = value;
  }

  public Command ElevatorTestDynamic(SysIdRoutine.Direction direction) {
    return routineToApply.dynamic(direction);
  }

  public Command ElevatorTestQuasistatic(SysIdRoutine.Direction direction) {
    return routineToApply.quasistatic(direction);
  }
}
