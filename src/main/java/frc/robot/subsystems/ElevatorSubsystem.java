package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leader = new TalonFX(ElevatorConstants.leaderID, ElevatorConstants.canName);
  private final TalonFX follower =
      new TalonFX(ElevatorConstants.followerID, ElevatorConstants.canName);
  private final DigitalInput hallSensor = new DigitalInput(ElevatorConstants.hallSensorID);

  private TalonFXConfiguration leaderconfigs = new TalonFXConfiguration();
  private TalonFXConfiguration followerconfigs = new TalonFXConfiguration();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicTorqueCurrentFOC motionMagic =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private NeutralOut neutralOut = new NeutralOut();

  private double encoderOffset = 0.0;
  private double timer = -1.0;
  private boolean hasHomed = false;

  public ElevatorSubsystem() {
    leaderconfigs.MotorOutput.withInverted(
        ElevatorConstants.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);
    followerconfigs.MotorOutput.withInverted(
        ElevatorConstants.inverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    /*
    leaderconfigs.Slot0.kP = ElevatorConstants.kP;
    leaderconfigs.Slot0.kI = ElevatorConstants.kI;
    leaderconfigs.Slot0.kD = ElevatorConstants.kD;
    leaderconfigs.Slot0.kA = ElevatorConstants.kA;
    leaderconfigs.Slot0.kS = ElevatorConstants.kS;
    leaderconfigs.Slot0.kV = ElevatorConstants.kV;
    */
    followerconfigs.Slot1.kP = 2.0;
    leaderconfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.cruiseVelocity;
    leaderconfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.cruiseAcceleration;
    leaderconfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
    leaderconfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);
    followerconfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
    followerconfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);
    leaderconfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.9;
    leader.getTorqueCurrent().setUpdateFrequency(1000);
    leader.getClosedLoopReference().setUpdateFrequency(1000);
    follower.getTorqueCurrent().setUpdateFrequency(1000);
    follower.getClosedLoopReference().setUpdateFrequency(1000);
    // follower differential control
    followerconfigs.DifferentialConstants.PeakDifferentialDutyCycle =
        ElevatorConstants.forwardDutyCycleLimit;
    followerconfigs.DifferentialConstants.PeakDifferentialVoltage = 4.0;
    followerconfigs.DifferentialSensors.DifferentialSensorSource =
        DifferentialSensorSourceValue.RemoteTalonFX_Diff;
    followerconfigs.DifferentialSensors.DifferentialTalonFXSensorID = ElevatorConstants.leaderID;
    leader.getConfigurator().apply(leaderconfigs);
    follower.getConfigurator().apply(followerconfigs);
    // follower.setControl(new Follower(ElevatorConstants.leaderID,
    // ElevatorConstants.opposeMaster));
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  public void shutDown() {
    leader.setControl(neutralOut);
    follower.setControl(neutralOut);
  }

  public void manualUpVolts() {
    leader.setControl(voltageOut.withOutput(ElevatorConstants.manualUpVoltage));
    follower.setControl(
        new DifferentialVoltage(ElevatorConstants.manualUpVoltage, 0.0).withDifferentialSlot(1));
  }

  public void manualDownVolts() {
    leader.setControl(voltageOut.withOutput(ElevatorConstants.manualDownVoltage));
    follower.setControl(
        new DifferentialVoltage(ElevatorConstants.manualDownVoltage, 0.0).withDifferentialSlot(1));
  }

  public void lockVolts() {
    leader.setControl(voltageOut.withOutput(ElevatorConstants.lockVoltage));
    follower.setControl(
        new DifferentialVoltage(ElevatorConstants.lockVoltage, 0.0).withDifferentialSlot(1));
  }

  public void setHeight(double height) {
    height += encoderOffset;
    if (Math.abs(leader.getPosition().getValueAsDouble() - height)
        < ElevatorConstants.positionDeadband) {
      lockVolts();
    } else if (leader.getPosition().getValueAsDouble() > height) {
      manualDownVolts();
    } else {
      manualUpVolts();
    }
  }

  public boolean isAtPosition(double height) {
    height += encoderOffset;
    return Math.abs(leader.getPosition().getValueAsDouble() - height)
        < ElevatorConstants.positionDeadband;
  }

  public void home() {
    if (timer == -1) {
      SmartDashboard.putNumber("temp", 33);
      timer = Timer.getFPGATimestamp();
    }

    if (Timer.getFPGATimestamp() - timer < ElevatorConstants.homeUpTime) {
      manualUpVolts();
    } else {
      manualDownVolts();
      if (getHallSensorActive()) {
        resetOffset();
        shutDown();
        hasHomed = true;
      }
    }
  }

  public void setHasHomed(boolean set) {
    hasHomed = set;
  }

  public boolean getHasHomed() {
    return hasHomed;
  }

  public void resetTimer() {
    timer = -1;
  }

  public void set2L1() {
    report();
    setHeight(ElevatorConstants.L1Position);
  }

  public void set2L2() {
    report();
    setHeight(ElevatorConstants.L2Position);
  }

  public void set2L3() {
    report();
    setHeight(ElevatorConstants.L3Position);
  }

  public void set2L4() {
    report();
    setHeight(ElevatorConstants.L4Position);
  }

  public void hold() {
    follower.setControl(new Follower(ElevatorConstants.leaderID, ElevatorConstants.opposeMaster));
    leader.setControl(
        motionMagic
            .withPosition(leader.getPosition().getValueAsDouble())
            .withFeedForward(ElevatorConstants.kF));
  }

  public void resetOffset() {
    encoderOffset = leader.getPosition().getValueAsDouble();
  }

  public boolean getHallSensorActive() {
    return !hallSensor.get();
  }

  public void report() {
    SmartDashboard.putNumber("elevator/leader position", leader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/follower position", follower.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/position difference",
        leader.getPosition().getValueAsDouble() - follower.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("elevator/hallsensor", !hallSensor.get());
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("elevator/encoder offset", encoderOffset);
    SmartDashboard.putNumber("elevator/leader velocity", leader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/leader acceleration", leader.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/leader torquecurrent", leader.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/follower torquecurrent", follower.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/motionmagic target", leader.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("elevator/timer", timer);
    SmartDashboard.putBoolean("elevator/hasHomed", hasHomed);
    SmartDashboard.putNumber("elevator/leaderVoltage", leader.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/followerVoltage", follower.getMotorVoltage().getValueAsDouble());
  }
}
