package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RollerConstants;
import frc.robot.util.MagicTimer;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final String name;
  private final RollerSystemIO io;
  private final Alert disconnected;

  private MagicTimer rollerTimer = new MagicTimer();
  private boolean isHoldAlgae = false;

  public RollerSubsystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;
    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  // for operator and test
  public void manualForwardVolts() {
    io.runVolts(RollerConstants.ROLLER_MANUALFORWARD_VOLTAGE);
  }

  // for operator and test
  public void manualReverseVolts() {
    io.runVolts(RollerConstants.ROLLER_MANUALINVERT_VOLTAGE);
  }

  // for test
  public void manualForwardVelocity() {
    io.runVelocity(RollerConstants.ROLLER_MANUALFORWARD_VOLTAGE);
  }

  // for test
  public void manualReverseVelocity() {
    io.runVelocity(RollerConstants.ROLLER_MANUALINVERT_VOLTAGE);
  }

  // for default command
  public void defaultIdleVelocity() {
    if (isHoldAlgae) {
      io.runVelocity(RollerConstants.ROLLER_ALGAE_IDLE_VELOCITY);
    } else {
      io.runVelocity(RollerConstants.ROLLER_CORAL_IDLE_VELOCITY);
    }
  }

  // for driver, spinning the roller until a certain time has passed since the canRange detected
  // coral;
  public void station() {
    if (isCanRangeActive()) {
      rollerTimer.startTimer();
      if (rollerTimer.getTimePassedSec() > RollerConstants.ROLLER_STATION_TIME) {
        io.runVelocity(RollerConstants.ROLLER_CORAL_IDLE_VELOCITY);
      } else {
        io.runVelocity(RollerConstants.ROLLER_LOAD_CORAL_VELOCITY);
      }
    } else {
      rollerTimer.resetTimer();
      io.runVelocity(RollerConstants.ROLLER_LOAD_CORAL_VELOCITY);
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (DriverStation.isDisabled()) {
      io.stop();
    }
  }

  public void L1() {
    io.runVelocity(RollerConstants.ROLLER_L1_VELOCITY);
  }

  public void L2() {
    io.runVelocity(RollerConstants.ROLLER_L2_VELOCITY);
  }

  public void L3() {
    io.runVelocity(RollerConstants.ROLLER_L3_VELOCITY);
  }

  public void L4() {
    io.runVelocity(RollerConstants.ROLLER_L4_VELOCITY);
  }

  public void intakeAlgaeFromReef() {
    io.runVelocity(RollerConstants.ROLLER_LOAD_ALGAE_VELOCITY);
  }

  public void processor() {
    io.runVelocity(RollerConstants.ROLLER_PROCESSOR_VELOCITY);
  }

  public boolean isCanRangeActive() {
    return io.isCanRangeTriggered();
  }

  // methods to manage isHoldAlgae boolean
  public boolean getHoldAlgae() {
    return isHoldAlgae;
  }

  public void setHoldAlgae(boolean set) {
    isHoldAlgae = set;
  }

  public void switchHoldAlgae() {
    isHoldAlgae = !isHoldAlgae;
  }

  public void shutDown() {
    io.stop();
  }
}
