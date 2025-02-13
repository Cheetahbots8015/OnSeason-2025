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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.ElevatorConstants;
import frc.robot.util.NarcissusUtil;
import org.littletonrobotics.junction.Logger;

public class ElevatorSystem extends SubsystemBase {
  private final String name;
  private final ElevatorSystemIO io;
  protected final ElevatorSystemIOInputsAutoLogged inputs = new ElevatorSystemIOInputsAutoLogged();
  private final Alert disconnected;

  private ElevatorState systemState = ElevatorState.IDLE;
  private ElevatorRequest elevatorRequest = ElevatorRequest.NULL;
  private ElevatorPositionTarget elevatorPositionTarget = ElevatorPositionTarget.NULL;

  private double manualVoltage = 0.0;
  private boolean useManualDynamic = false;
  private boolean usePositionDynamic = false;

  private boolean homed = false;
  private double homeTimer = -1.0;

  /* System States */
  private enum ElevatorState {
    IDLE,
    HOME_UP,
    HOME_DOWN,
    MANUAL,
    POSITION
  }

  public enum ElevatorPositionTarget {
    NULL,
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
  }

  public enum ElevatorRequest {
    NULL,
    HOME,
    POSITION,
    MANUAL,
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
    SmartDashboard.putString("state", systemState.toString());
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs(name, inputs);
    Logger.recordOutput(name, this.getSystemState());
    disconnected.set(!inputs.connected);
    updateStateMachine();
    if (DriverStation.isDisabled()) {
      io.stop();
    }
  }

  public void updateStateMachine() {
    SmartDashboard.putBoolean("L2 position boolean", false);
    if (getNewState() != systemState) {
      switch (getNewState()) {
        case IDLE:
          homeTimer = -1.0;
          setSoftLimitEnable(true, true);
          break;

        case HOME_UP:
          setSoftLimitEnable(false, true);
          break;

        case HOME_DOWN:
          setSoftLimitEnable(true, false);
          break;

        case MANUAL:
          homeTimer = -1.0;
          homed = false;
          setSoftLimitEnable(false, false);
          break;

        case POSITION:
          homeTimer = -1.0;
          setSoftLimitEnable(true, true);
          break;

        default:
          break;
      }
      systemState = getNewState();
    }

    switch (systemState) {
      case IDLE:
        if (homed) {
          hold();
        } else {
          io.stop();
        }
        break;

      case MANUAL:
        io.setVolts(manualVoltage);
        break;

      case POSITION:
        switch (elevatorPositionTarget) {
          case HOME:
            if (isAtPosition(ElevatorConstants.ELEVATOR_HOME_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_HOME_POSITION_RADS);
            }
            break;

          case L1:
            if (isAtPosition(ElevatorConstants.ELEVATOR_L1_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_L1_POSITION_RADS);
            }
            break;

          case L2:
            SmartDashboard.putBoolean("L2 position boolean", true);
            SmartDashboard.putBoolean(
                "isatposition", isAtPosition(ElevatorConstants.ELEVATOR_L2_POSITION_RADS));

            if (isAtPosition(ElevatorConstants.ELEVATOR_L2_POSITION_RADS)) {
              io.stop();
              // hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_L2_POSITION_RADS);
            }
            break;

          case L3:
            if (isAtPosition(ElevatorConstants.ELEVATOR_L3_POSITION_RADS)) {
              io.stop();
              // hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_L3_POSITION_RADS);
            }
            break;

          case L4:
            if (isAtPosition(ElevatorConstants.ELEVATOR_L4_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_L4_POSITION_RADS);
            }
            break;

          case LOW_ALGAE:
            if (isAtPosition(ElevatorConstants.ELEVATOR_LOW_ALGAE_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_LOW_ALGAE_POSITION_RADS);
            }
            break;

          case HIGH_ALGAE:
            if (isAtPosition(ElevatorConstants.ELEVATOR_HIGH_ALGAE_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_HIGH_ALGAE_POSITION_RADS);
            }
            break;

          case PROCESSOR:
            if (isAtPosition(ElevatorConstants.ELEVATOR_PROCESSOR_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_PROCESSOR_POSITION_RADS);
            }
            break;

          case BARGE:
            if (isAtPosition(ElevatorConstants.ELEVATOR_BARGE_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_BARGE_POSITION_RADS);
            }
            break;

          case STATION:
            if (isAtPosition(ElevatorConstants.ELEVATOR_STATION_POSITION_RADS)) {
              hold();
            } else {
              set2Position(ElevatorConstants.ELEVATOR_STATION_POSITION_RADS);
            }
            break;

          case NULL:
            io.stop();

          default:
            break;
        }
        break;

      case HOME_UP:
        io.setVolts(ElevatorConstants.ELEVATOR_HOME_UP_VOLTAGE);
        break;

      case HOME_DOWN:
        if (io.isHallSensorActive()) {
          io.setEncoder2Zero();
          io.stop();
          homed = true;
        } else {
          io.setVolts(ElevatorConstants.ELEVATOR_HOME_DOWN_VOLTAGE);
        }
        break;

      default:
        break;
    }
  }

  public void setRequest(ElevatorRequest request) {
    elevatorRequest = request;
  }

  public void setUseManualDynamic(boolean set) {
    useManualDynamic = set;
  }

  public void setManualVoltage(double value) {
    manualVoltage = value;
  }

  public void setPositionTarget(ElevatorPositionTarget target) {
    elevatorPositionTarget = target;
  }

  public void setUsePositionDynamic(boolean set) {
    usePositionDynamic = set;
  }

  /* Returns the current system state of the elevator */
  public ElevatorState getSystemState() {
    return systemState;
  }

  private ElevatorState getNewState() {

    if (elevatorRequest == ElevatorRequest.NULL) {
      return ElevatorState.IDLE;
    }
    if (systemState == ElevatorState.HOME_UP) {
      if (Timer.getFPGATimestamp() - homeTimer >= ElevatorConstants.ELEVATOR_HOME_UP_TIME) {
        systemState = ElevatorState.HOME_DOWN;
      }
    }
    if (systemState != ElevatorState.IDLE) {
      return systemState;
    } else {
      switch (elevatorRequest) {
        case HOME:
          if (homeTimer == -1) {
            homeTimer = Timer.getFPGATimestamp();
          }
          if (Timer.getFPGATimestamp() - homeTimer < ElevatorConstants.ELEVATOR_HOME_UP_TIME) {
            return ElevatorState.HOME_UP;
          } else {
            return ElevatorState.HOME_DOWN;
          }

        case POSITION:
          if (homed) {
            return ElevatorState.POSITION;
          } else {
            return ElevatorState.IDLE;
          }

        case MANUAL:
          return ElevatorState.MANUAL;

        default:
          return ElevatorState.IDLE;
      }
    }
  }

  public Command ElevatorTestDynamic(SysIdRoutine.Direction direction) {
    return routineToApply.dynamic(direction);
  }

  public Command ElevatorTestQuasistatic(SysIdRoutine.Direction direction) {
    return routineToApply.quasistatic(direction);
  }

  private void setSoftLimitEnable(boolean enableForward, boolean enableReverse) {
    // io.setSoftLimits(enableForward, enableReverse);
  }

  public void testCommand() {
    io.setEncoder2Zero();
    homed = true;
  }

  private double getEncoderPositionRads() {
    return inputs.positionRads;
  }

  private void set2Position(double target) {
    io.setHeightRads(target);
    if (NarcissusUtil.deadband(
            getEncoderPositionRads() - target,
            ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
        == 0) {
      io.stop();
    }
  }

  private boolean isAtPosition(double target) {
    return NarcissusUtil.deadband(
            getEncoderPositionRads() - target,
            ElevatorConstants.ELEVATOR_SET_POSITION_TOLERANCE_RADS)
        == 0;
  }

  private void hold() {
    // io.setHeightRads(getEncoderPositionRads());
  }
}
