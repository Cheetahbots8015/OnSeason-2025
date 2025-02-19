package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  private final TalonFX pivot = new TalonFX(PivotConstants.pivotID, PivotConstants.canName);
  private final CANdi candi = new CANdi(PivotConstants.candiID, PivotConstants.canName);

  private double offset;

  private TalonFXConfiguration pivotconfigs = new TalonFXConfiguration();
  private CANdiConfiguration candiconfigs = new CANdiConfiguration();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private NeutralOut neutralOut = new NeutralOut();

  private final SysIdRoutine m_SysIdRoutinePivot =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(1.5),
              Seconds.of(3.5),
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                pivot.setVoltage(volts.magnitude() * 0.5);
              },
              null,
              this));

  private final SysIdRoutine routineToApply = m_SysIdRoutinePivot;

  public PivotSubsystem() {
    pivotconfigs.MotorOutput.withInverted(
        PivotConstants.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);
    pivotconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotconfigs.Slot0.kP = PivotConstants.kP;
    pivotconfigs.Slot0.kI = PivotConstants.kI;
    pivotconfigs.Slot0.kD = PivotConstants.kD;
    pivotconfigs.Slot0.kA = PivotConstants.kA;
    pivotconfigs.Slot0.kS = PivotConstants.kS;
    pivotconfigs.Slot0.kV = PivotConstants.kV;
    pivotconfigs.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.cruiseVelocity;
    pivotconfigs.MotionMagic.MotionMagicAcceleration = PivotConstants.cruiseAcceleration;
    pivotconfigs.MotorOutput.withPeakForwardDutyCycle(PivotConstants.forwardDutyCycleLimit);
    pivotconfigs.MotorOutput.withPeakReverseDutyCycle(PivotConstants.reverseDutyCycleLimit);
    pivotconfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = PivotConstants.forwardSoftLimitEnable;
    pivotconfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = PivotConstants.reverseSoftLimitEnable;
    pivotconfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.forwardSoftLimitThreshold + candi.getPWM1Position().getValueAsDouble();
    pivotconfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.reverseSoftLimitThreshold + candi.getPWM1Position().getValueAsDouble();
    pivotconfigs.Feedback.FeedbackRemoteSensorID = PivotConstants.candiID;
    pivotconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANdiPWM1;
    pivotconfigs.Feedback.SensorToMechanismRatio = PivotConstants.candi2MechanismRatio;
    pivotconfigs.Feedback.RotorToSensorRatio = PivotConstants.rotor2CandiRatio;
    pivot.getConfigurator().apply(pivotconfigs);

    candiconfigs.PWM1.SensorDirection = PivotConstants.candiDirection;
    // candiconfigs.PWM1.AbsoluteSensorDiscontinuityPoint = 1;//Setting this to 1 makes the absolute
    // position unsigned [0, 1)
    candi.getConfigurator().apply(candiconfigs);

    offset = candi.getPWM1Position().getValueAsDouble();

    BaseStatusSignal.setUpdateFrequencyForAll(
        250, pivot.getPosition(), pivot.getVelocity(), pivot.getMotorVoltage());

    SignalLogger.start();
  }

  public void shutDown() {
    pivot.setControl(neutralOut);
  }

  public void manualVoltsForward() {
    report();
    pivot.setControl(
        voltageOut
            .withOutput(PivotConstants.manualVoltageForward)
            .withLimitForwardMotion(getPosition() > PivotConstants.forwardSoftLimitThreshold)
            .withLimitReverseMotion(getPosition() < PivotConstants.reverseSoftLimitThreshold));
  }

  public void manualVoltsReverse() {
    report();
    pivot.setControl(
        voltageOut
            .withOutput(PivotConstants.manualVoltageReverse)
            .withLimitForwardMotion(getPosition() > PivotConstants.forwardSoftLimitThreshold)
            .withLimitReverseMotion(getPosition() < PivotConstants.reverseSoftLimitThreshold));
  }

  public void setHeight(double height) {
    report();
    pivot.setControl(
        motionMagic
            .withPosition(height + offset)
            .withLimitForwardMotion(getPosition() > PivotConstants.forwardSoftLimitThreshold)
            .withLimitReverseMotion(getPosition() < PivotConstants.reverseSoftLimitThreshold));
  }

  public double getPosition() {
    return candi.getPWM1Position().getValueAsDouble() - offset;
  }

  public void set2Home() {
    setHeight(PivotConstants.HomePosition);
  }

  public void set2L1() {
    setHeight(PivotConstants.L1Position);
  }

  public void set2L2() {
    setHeight(PivotConstants.L2Position);
  }

  public void set2L3() {
    setHeight(PivotConstants.L3Position);
  }

  public void set2L4() {
    setHeight(PivotConstants.L4Position);
  }

  public void set2lolipop() {
    setHeight(PivotConstants.lolipopPosition);
  }

  public void set2LowAlgae() {
    setHeight(PivotConstants.lowAlgaePosition);
  }

  public void set2HighAlgae() {
    setHeight(PivotConstants.highAlgaePosition);
  }

  public void home() {
    setHeight(0.0);
  }

  public void algaeHome() {
    setHeight(PivotConstants.algaeHome);
  }

  public void hold() {
    report();
    pivot.setControl(
        motionMagic
            .withPosition(candi.getPWM1Position().getValueAsDouble())
            .withLimitForwardMotion(getPosition() > PivotConstants.forwardSoftLimitThreshold)
            .withLimitReverseMotion(getPosition() < PivotConstants.reverseSoftLimitThreshold));
  }

  public boolean isAtPosition(double height) {
    if (Math.abs(this.getPosition() - height) < PivotConstants.positionDeadband) {
      SmartDashboard.putBoolean("pivot/is at position", true);
    } else {
      SmartDashboard.putBoolean("pivot/is at position", false);
    }
    return Math.abs(this.getPosition() - height) < PivotConstants.positionDeadband;
  }

  public void report() {
    SmartDashboard.putNumber("pivot/pivot position", pivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("pivot/pivot velocity", pivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "pivot/pivot acceleration", pivot.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber(
        "pivot/pivot torquecurrent", pivot.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "pivot/motionmagic target", pivot.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("pivot/s1 position", candi.getPWM1Position().getValueAsDouble());
    SmartDashboard.putNumber("pivot/offset", offset);
  }

  public Command PivotTestDynamic(SysIdRoutine.Direction direction) {
    return routineToApply.dynamic(direction);
  }

  public Command PivotTestQuasistatic(SysIdRoutine.Direction direction) {
    return routineToApply.quasistatic(direction);
  }
}
