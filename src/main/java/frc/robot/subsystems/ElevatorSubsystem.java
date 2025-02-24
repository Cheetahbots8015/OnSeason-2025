package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ElevatorConstants;
import frc.robot.util.MagicTimer;

public class ElevatorSubsystem extends SubsystemBase {
  // krakenX60 initialization with configs
  private final TalonFX leader = new TalonFX(ElevatorConstants.leaderID, ElevatorConstants.canName);
  private final TalonFX follower =
      new TalonFX(ElevatorConstants.followerID, ElevatorConstants.canName);
  private TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
  private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();

  // hall sensor initialization
  private final DigitalInput hallSensor = new DigitalInput(ElevatorConstants.hallSensorID);

  // control methods initialization
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(50);
  private MotionMagicTorqueCurrentFOC motionMagic =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(200);
  private NeutralOut neutralOut = new NeutralOut();

  // used to zero the krankenX60s
  private double leaderEncoderOffset = 0.0;
  private double followerEncoderOffset = 0.0;
  private MagicTimer elevatorTimer = new MagicTimer();

  private MedianFilter filter = new MedianFilter(30);

  // at the beginning of homing process, the phase should be phase1
  // if the elevator is in phase1 and hall sensor is active, then switch to phase2
  // otherwise, if the elevator is in phase2 and hall sensor is not active, the
  // switch to phase3
  // elevator in phase2 will move up and the move down until hall sensor is active
  // again
  // elevator in phase3 will directly move down until hall sensor is active
  // by the end of homing process, the phase should be set back to phase1
  private enum homePhase {
    phase1,
    phase2,
    phase3
  }

  private homePhase systemHomePhase = homePhase.phase1;

  public ElevatorSubsystem() {
    // config neutralmode
    leaderConfigs.MotorOutput.withNeutralMode(
        ElevatorConstants.leader_Neutalmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    followerConfigs.MotorOutput.withNeutralMode(
        ElevatorConstants.leader_Neutalmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    // config direction
    // POSITIVE for leader moving up and NEGATIVE for follower moving up
    leaderConfigs.MotorOutput.withInverted(
        ElevatorConstants.leader_Inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    followerConfigs.MotorOutput.withInverted(
        ElevatorConstants.follower_Inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // config PIDSAV
    leaderConfigs.Slot0.kP = ElevatorConstants.leader_kP;
    leaderConfigs.Slot0.kI = ElevatorConstants.leader_kI;
    leaderConfigs.Slot0.kD = ElevatorConstants.leader_kD;
    leaderConfigs.Slot0.kS = ElevatorConstants.leader_kS;
    leaderConfigs.Slot0.kA = ElevatorConstants.leader_kA;
    leaderConfigs.Slot0.kV = ElevatorConstants.leader_kV;
    followerConfigs.Slot0.kP = ElevatorConstants.follower_kP;
    followerConfigs.Slot0.kI = ElevatorConstants.follower_kI;
    followerConfigs.Slot0.kD = ElevatorConstants.follower_kD;
    followerConfigs.Slot0.kS = ElevatorConstants.follower_kS;
    followerConfigs.Slot0.kA = ElevatorConstants.follower_kA;
    followerConfigs.Slot0.kV = ElevatorConstants.follower_kV;

    // config motionmagic
    leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.leader_CruiseVelocity;
    leaderConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.leader_CruiseAcceleration;
    followerConfigs.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.follower_CruiseVelocity;
    followerConfigs.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.follower_CruiseAcceleration;

    // config ducy cycle limit
    leaderConfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
    leaderConfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);
    followerConfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
    followerConfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);

    // config softlimit
    // hall sensor is used as the reverse limit
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        ElevatorConstants.leader_ForwardSoftLimitEnable;
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.leader_forwardSoftLimitThreshold
            + leader.getPosition().getValueAsDouble();
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        ElevatorConstants.follower_ForwardSoftLimitEnable;
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.follower_forwardSoftLimitThreshold
            + follower.getPosition().getValueAsDouble();

    // follower differential control
    followerConfigs.Slot1.kP = ElevatorConstants.differentialkP;
    followerConfigs.DifferentialConstants.PeakDifferentialDutyCycle =
        ElevatorConstants.forwardDutyCycleLimit;
    followerConfigs.DifferentialConstants.PeakDifferentialVoltage =
        ElevatorConstants.peakDifferentialVoltage;
    followerConfigs.DifferentialSensors.DifferentialSensorSource =
        DifferentialSensorSourceValue.RemoteTalonFX_Diff;
    followerConfigs.DifferentialSensors.DifferentialTalonFXSensorID = ElevatorConstants.leaderID;

    // apply configs
    leader.getConfigurator().apply(leaderConfigs);
    follower.getConfigurator().apply(followerConfigs);

    // initialize offset
    leaderEncoderOffset = leader.getPosition().getValueAsDouble();
    followerEncoderOffset = follower.getPosition().getValueAsDouble();
  }

  public double getLeaderPositionWithoutOffset() {
    return leader.getPosition().getValueAsDouble() - leaderEncoderOffset;
  }

  public double getFollowerPositionWithoutOffset() {
    return follower.getPosition().getValueAsDouble() - followerEncoderOffset;
  }

  public void updateOffsets() {
    leaderEncoderOffset = leader.getPosition().getValueAsDouble();
    followerEncoderOffset = follower.getPosition().getValueAsDouble();
    leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.leader_forwardSoftLimitThreshold
            + leader.getPosition().getValueAsDouble();
    followerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.follower_forwardSoftLimitThreshold
            + follower.getPosition().getValueAsDouble();
    leader.getConfigurator().apply(leaderConfigs);
    follower.getConfigurator().apply(followerConfigs);
  }

  // reset the phase to phase1
  public void resetHomePhase() {
    systemHomePhase = homePhase.phase1;
  }

  // shutdown krakenX60s
  public void shutDown() {
    leader.setControl(neutralOut);
    follower.setControl(neutralOut);
  }

  // basic function to run voltage out, should be used by multiple functions
  public void setVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts).withLimitReverseMotion(getHallSensorActive()));
    follower.setControl(
        new DifferentialVoltage(volts, 0.0)
            .withDifferentialSlot(1)
            .withUpdateFreqHz(50.0)
            .withLimitReverseMotion(getHallSensorActive()));
  }

  // used in default command to let the elevator go to its home position(lowest
  // position) quickly
  // give a negative voltage if the elevator is high and let it free fall when it
  // is low
  public void defaultDown() {
    if (this.getLeaderPositionWithoutOffset() > ElevatorConstants.defaultDownShutDownPosition) {
      setVolts(ElevatorConstants.defaultDownVoltage);
    } else {
      shutDown();
    }
  }

  public void manualVoltsUp() {
    setVolts(ElevatorConstants.manualUpVoltage);
  }

  public void manualVoltsDown() {
    setVolts(ElevatorConstants.manualDownVoltage);
  }

  public void hold() {
    setVolts(ElevatorConstants.holdVoltage);
  }

  public void setPosition_MotionMagic(double position) {
    leader.setControl(
        motionMagic
            .withPosition(position + leaderEncoderOffset)
            .withLimitReverseMotion(getHallSensorActive())
            .withLimitForwardMotion(
                this.getLeaderPositionWithoutOffset()
                    > ElevatorConstants.leader_forwardSoftLimitThreshold));
    follower.setControl(
        motionMagic
            .withPosition(position + followerEncoderOffset)
            .withLimitReverseMotion(getHallSensorActive())
            .withLimitForwardMotion(
                this.getFollowerPositionWithoutOffset()
                    > ElevatorConstants.follower_forwardSoftLimitThreshold));
  }

  // use median filter to get smooth velocity curves for long distance movement
  public void setPosition_MotionMagician(double position) {
    if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
        < ElevatorConstants.positionDeadband) {
      setVolts(ElevatorConstants.holdVoltage);
    } else if (this.getLeaderPositionWithoutOffset()
        > position + ElevatorConstants.motionmagicianClosePositionDeadband_L3L4) {
      setVolts(filter.calculate(ElevatorConstants.motionmagicianHighDownVoltage_L3L4));
    } else if (this.getLeaderPositionWithoutOffset()
        < position - ElevatorConstants.motionmagicianClosePositionDeadband_L3L4) {
      setVolts(filter.calculate(ElevatorConstants.motionmagicianHighUpVoltage_L3L4));
    } else if (this.getLeaderPositionWithoutOffset() > position) {
      setVolts(filter.calculate(ElevatorConstants.motionmagicianLowDownVoltage_L3L4));
    } else {
      setVolts(filter.calculate(ElevatorConstants.motionmagicianLowUpVoltage_L3L4));
    }
  }

  // no need to use median filter for short distance movement
  public void setPosition_MotionMagicianLow(double position) {
    if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
        < ElevatorConstants.positionDeadband) {
      setVolts(ElevatorConstants.holdVoltage);
    } else if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
            < ElevatorConstants.motionmagicianClosePositionDeadband_L1L2
        && this.getLeaderPositionWithoutOffset() > position) {
      setVolts(ElevatorConstants.motionmagicianLowDownVoltage_L1L2);
    } else if (Math.abs(this.getLeaderPositionWithoutOffset() - position)
            < ElevatorConstants.motionmagicianClosePositionDeadband_L1L2
        && this.getLeaderPositionWithoutOffset() <= position) {
      setVolts(ElevatorConstants.motionmagicianLowUpVoltage_L1L2);
    } else if (this.getLeaderPositionWithoutOffset() > position) {
      setVolts(ElevatorConstants.motionmagicianHighDownVoltage_L1L2);
    } else {
      setVolts(ElevatorConstants.motionmagicianHighUpVoltage_L1L2);
    }
  }

  public void resetFilter() {
    filter.reset();
  }

  public boolean isAtPosition(double height) {
    if (Math.abs(this.getFollowerPositionWithoutOffset() - height)
        < ElevatorConstants.positionDeadband) {
      SmartDashboard.putBoolean("elevator/is at position", true);
    } else {
      SmartDashboard.putBoolean("elevator/is at position", false);
    }
    return Math.abs(this.getFollowerPositionWithoutOffset() - height)
        < ElevatorConstants.positionDeadband;
  }

  public boolean isAbovePosition(double height) {
    return this.getLeaderPositionWithoutOffset() > height;
  }

  public void home() {
    if (systemHomePhase == homePhase.phase1) {
      if (getHallSensorActive()) {
        elevatorTimer.resetTimer();
        elevatorTimer.startTimer();
        systemHomePhase = homePhase.phase2;
      }
    } else if (systemHomePhase == homePhase.phase2) {
      if (elevatorTimer.getTimePassedSec() < ElevatorConstants.homeUpTime) {
        setVolts(ElevatorConstants.homeUpVoltage);
      } else {
        if (getHallSensorActive()) {
          updateOffsets();
          resetHomePhase();
          shutDown();
        } else {
          setVolts(ElevatorConstants.homeDownVoltage);
        }
      }
    } else if (systemHomePhase == homePhase.phase3) {
      if (getHallSensorActive()) {
        updateOffsets();
        resetHomePhase();
        shutDown();
      } else {
        setVolts(ElevatorConstants.homeDownVoltage);
      }
    }
  }

  public void L1() {
    setPosition_MotionMagicianLow(ElevatorConstants.L1Position);
  }

  public void L2() {
    setPosition_MotionMagicianLow(ElevatorConstants.L2Position);
  }

  public void L3() {
    setPosition_MotionMagicianLow(ElevatorConstants.L3Position);
  }

  public void L4() {
    setPosition_MotionMagicianLow(ElevatorConstants.L4Position);
  }

  public void lowAlgae() {
    setPosition_MotionMagician(ElevatorConstants.lowAlgaePosition);
  }

  public void highAlgae() {
    setPosition_MotionMagician(ElevatorConstants.highAlgaePosition);
  }

  public boolean getHallSensorActive() {
    return !hallSensor.get();
  }

  public void report() {
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("elevator/leader position", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/follower position", follower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/position difference",
        leader.getPosition().getValueAsDouble() - follower.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("elevator/hallsensor", getHallSensorActive());
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("elevator/leader encoder offset", leaderEncoderOffset);
    SmartDashboard.putNumber("elevator/follower encoder offset", followerEncoderOffset);
    SmartDashboard.putNumber("elevator/leader velocity", leader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/leader acceleration", leader.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/leader torquecurrent", leader.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/follower torquecurrent", follower.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/motionmagic target", leader.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("elevator/leaderVoltage", leader.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/followerVoltage", follower.getMotorVoltage().getValueAsDouble());
  }
}
