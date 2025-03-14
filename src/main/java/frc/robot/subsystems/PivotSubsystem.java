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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  // krakenX60 initialization with configs
  private final TalonFX pivot = new TalonFX(PivotConstants.pivotID, PivotConstants.canName);
  private TalonFXConfiguration pivotconfigs = new TalonFXConfiguration();

  // candi initialization with configs
  private final CANdi candi = new CANdi(PivotConstants.candiID, PivotConstants.canName);
  private CANdiConfiguration candiconfigs = new CANdiConfiguration();

  // control methods initialization
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private NeutralOut neutralOut = new NeutralOut();

  // offset of candi, used to match relative encoder
  private double offset;

  // indicate whether holding algae, defaultly regarded as holding coral since the
  // idle velocity of holding coral is zero
  public enum pivotIdleState {
    coral,
    algae,
    manual
  }

  private pivotIdleState systemIdleState = pivotIdleState.coral;

  private DoubleSubscriber elevatorPositionDoubleSubscriber;

  // sysID stuffs
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
    // config neutralmode
    pivotconfigs.MotorOutput.withNeutralMode(
        PivotConstants.neutalmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    // config direction
    // POSITIVE for rotating outward
    pivotconfigs.MotorOutput.withInverted(
        PivotConstants.inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    // config PIDSAV
    pivotconfigs.Slot0.kP = PivotConstants.kP;
    pivotconfigs.Slot0.kI = PivotConstants.kI;
    pivotconfigs.Slot0.kD = PivotConstants.kD;
    pivotconfigs.Slot0.kA = PivotConstants.kA;
    pivotconfigs.Slot0.kS = PivotConstants.kS;
    pivotconfigs.Slot0.kV = PivotConstants.kV;
    // vonfig motion magic
    pivotconfigs.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.cruiseVelocity;
    pivotconfigs.MotionMagic.MotionMagicAcceleration = PivotConstants.cruiseAcceleration;
    // config duty cycle limit
    pivotconfigs.MotorOutput.withPeakForwardDutyCycle(PivotConstants.forwardDutyCycleLimit);
    pivotconfigs.MotorOutput.withPeakReverseDutyCycle(PivotConstants.reverseDutyCycleLimit);
    // config softlimit
    pivotconfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = PivotConstants.forwardSoftLimitEnable;
    pivotconfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = PivotConstants.reverseSoftLimitEnable;
    pivotconfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.forwardSoftLimitThreshold + candi.getPWM1Position().getValueAsDouble();
    pivotconfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.reverseSoftLimitThreshold + candi.getPWM1Position().getValueAsDouble();
    // config remote feedback sensor
    pivotconfigs.Feedback.FeedbackRemoteSensorID = PivotConstants.candiID;
    pivotconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANdiPWM1;
    pivotconfigs.Feedback.SensorToMechanismRatio = PivotConstants.candi2MechanismRatio;
    pivotconfigs.Feedback.RotorToSensorRatio = PivotConstants.rotor2CandiRatio;
    // apply configs
    pivot.getConfigurator().apply(pivotconfigs);

    candiconfigs.PWM1.SensorDirection = PivotConstants.candiDirection;
    // apply configs
    candi.getConfigurator().apply(candiconfigs);

    // initialize offset
    offset = candi.getPWM1Position().getValueAsDouble();

    // start logging for sysID
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, pivot.getPosition(), pivot.getVelocity(), pivot.getMotorVoltage());

    SignalLogger.start();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    elevatorPositionDoubleSubscriber = table.getDoubleTopic("elevator/position").subscribe(0.0);
  }

  // shutDown krankenX60
  public void shutDown() {
    pivot.setControl(neutralOut);
  }

  // basic function to run voltage out, should be used by multiple functions
  public void setVolts(double volts) {
    report();
    pivot.setControl(
        voltageOut
            .withOutput(volts)
            .withLimitForwardMotion(
                getPositionwithoutOffset() > PivotConstants.forwardSoftLimitThreshold)
            .withLimitReverseMotion(
                getPositionwithoutOffset() < PivotConstants.reverseSoftLimitThreshold));
  }

  // basic function to run motion magic, should be used by multiple functions
  public void setPosition(double height) {
    report();
    pivot.setControl(
        motionMagic
            .withPosition(height + offset)
            .withLimitForwardMotion(
                getPositionwithoutOffset() > PivotConstants.forwardSoftLimitThreshold)
            .withLimitReverseMotion(
                getPositionwithoutOffset() < PivotConstants.reverseSoftLimitThreshold));
  }

  // for operator and test
  public void manualVoltsForward() {
    setVolts(PivotConstants.manualForwardVoltage);
  }

  // for operator and test
  public void manualVoltsReverse() {
    setVolts(PivotConstants.manualReverseVoltage);
  }

  public double getPositionwithoutOffset() {
    return candi.getPWM1Position().getValueAsDouble() - offset;
  }

  public void idle() {
    report();
    if (systemIdleState == pivotIdleState.coral) {
      if (elevatorPositionDoubleSubscriber.get() > 30.0) {
        setPosition(0.05);
      } else {
        setPosition(PivotConstants.coralHomePosition);
      }
    } else if (systemIdleState == pivotIdleState.algae) {
      setPosition(PivotConstants.algaeHomePosition);
    } else {
      hold();
    }
  }

  public void station() {
    setPosition(-0.3);
  }

  public void L1() {
    setPosition(PivotConstants.L1Position);
  }

  public void L2() {
    setPosition(PivotConstants.L2Position);
  }

  public void L3() {
    setPosition(PivotConstants.L3Position);
  }

  public void L4() {
    setPosition(PivotConstants.L4Position);
  }

  public void lowAlgae() {
    setPosition(PivotConstants.lowAlgaePosition);
  }

  public void highAlgae() {
    setPosition(PivotConstants.highAlgaePosition);
  }

  public void processor() {
    setPosition(PivotConstants.processorPosition);
  }

  public void hold() {
    setPosition(this.getPositionwithoutOffset());
  }

  public boolean isAtPosition(double height) {
    if (Math.abs(this.getPositionwithoutOffset() - height) < PivotConstants.positionDeadband) {
      SmartDashboard.putBoolean("pivot/is at position", true);
    } else {
      SmartDashboard.putBoolean("pivot/is at position", false);
    }
    return Math.abs(this.getPositionwithoutOffset() - height) < PivotConstants.positionDeadband;
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
    SmartDashboard.putString("pivot/idle state", systemIdleState.toString());
  }

  // methods to manage system idle state
  public pivotIdleState getSysteIdleState() {
    return systemIdleState;
  }

  public void setSystemIdleState(pivotIdleState state) {
    systemIdleState = state;
  }

  public void switchIdleState() {
    if (systemIdleState == pivotIdleState.algae) {
      systemIdleState = pivotIdleState.coral;
    } else if (systemIdleState == pivotIdleState.coral) {
      systemIdleState = pivotIdleState.algae;
    } else {
      systemIdleState = pivotIdleState.coral;
    }
  }

  // sysID command
  public Command PivotTestDynamic(SysIdRoutine.Direction direction) {
    return routineToApply.dynamic(direction);
  }

  // sysID command
  public Command PivotTestQuasistatic(SysIdRoutine.Direction direction) {
    return routineToApply.quasistatic(direction);
  }
}
