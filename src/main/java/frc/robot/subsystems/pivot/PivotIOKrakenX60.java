package frc.robot.subsystems.pivot;

import static frc.robot.constants.PivotConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.RollerConstants;

public class PivotIOKrakenX60 implements PivotIO {

  /* Hardware */
  private final TalonFX pivot;
  private final CANdi candi;

  /* Signals */
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Double> motionMagicTarget;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Angle> s1Position;

  /* Configurators */
  private TalonFXConfiguration pivotConfiguration;
  private CANdiConfiguration candiConfiguration;

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private NeutralOut neutralOut = new NeutralOut();

  public PivotIOKrakenX60() {
    this.pivot = new TalonFX(PIVOT_ID, PIVOT_CANNAME);
    this.candi = new CANdi(CANDI_ID, PIVOT_CANNAME);

    this.pivotConfiguration = new TalonFXConfiguration();
    this.candiConfiguration = new CANdiConfiguration();

    pivotConfiguration.MotorOutput.withInverted(
        PIVOT_INVERSION
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);
    pivotConfiguration.Slot0.kP = PIVOT_KP;
    pivotConfiguration.Slot0.kI = PIVOT_KI;
    pivotConfiguration.Slot0.kD = PIVOT_KD;
    pivotConfiguration.Slot0.kA = PIVOT_KA;
    pivotConfiguration.Slot0.kS = PIVOT_KS;
    pivotConfiguration.Slot0.kV = PIVOT_KV;
    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = PIVOT_MOTION_MAGIC_CRUISE_VELOCITY;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration = PIVOT_MOTION_MAGIC_ACCELERATION;
    pivotConfiguration.MotorOutput.withPeakForwardDutyCycle(PIVOT_FORWARD_DUTY_CYCLE_LIMIT);
    pivotConfiguration.MotorOutput.withPeakReverseDutyCycle(PIVOT_REVERSE_SOFT_LIMIT_THRESHOLD);
    pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = PIVOT_FORWARD_SOFT_LIMIT_ENABLE;
    pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = PIVOT_REVERSE_SOFT_LIMIT_ENABLE;
    pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD;
    pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD;
    pivotConfiguration.Feedback.FeedbackRemoteSensorID = CANDI_ID;
    pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
    pivotConfiguration.Feedback.SensorToMechanismRatio = PIVOT_CANDI_2_MECHANISM_RATIO;
    pivotConfiguration.Feedback.RotorToSensorRatio = PIVOT_ROTOR_2_CANDI_RATIO;
    pivot.getConfigurator().apply(pivotConfiguration);

    candiConfiguration.PWM1.SensorDirection = CANDI_DIRECTION;
    candiConfiguration.PWM1.AbsoluteSensorOffset = CANDI_OFFSET;
    candi.getConfigurator().apply(candiConfiguration);

    position = pivot.getPosition();
    velocity = pivot.getVelocity();
    acceleration = pivot.getAcceleration();
    torqueCurrent = pivot.getTorqueCurrent();
    motionMagicTarget = pivot.getClosedLoopReference();
    appliedVoltage = pivot.getMotorVoltage();
    tempCelsius = pivot.getDeviceTemp();
    s1Position = candi.getPWM1Position();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                RollerConstants.SIGNAL_UPDATE_FREQUENCY_HZ,
                position,
                velocity,
                acceleration,
                torqueCurrent,
                motionMagicTarget,
                appliedVoltage,
                tempCelsius,
                s1Position));
    tryUntilOk(5, () -> pivot.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, tempCelsius).isOK();
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.acceleration = acceleration.getValueAsDouble();
    inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
    inputs.motionMagicTarget = motionMagicTarget.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
    inputs.s1Position = s1Position.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    pivot.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setAngle(double height, double offset) {
    pivot.setControl(motionMagic.withPosition(height + offset));
  }

  @Override
  public void hold(double position) {
    pivot.setControl(motionMagic.withPosition(position));
  }
}
