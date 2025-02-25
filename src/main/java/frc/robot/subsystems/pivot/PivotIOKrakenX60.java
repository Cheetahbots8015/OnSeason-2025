package frc.robot.subsystems.pivot;

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
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.PivotConstants;

public class PivotIOKrakenX60 implements PivotIO {

  /* Hardware */
  private final TalonFX pivot = new TalonFX(PivotConstants.PIVOT_ID, PivotConstants.PIVOT_CANNAME);
  private final CANdi candi = new CANdi(PivotConstants.CANDI_ID, PivotConstants.PIVOT_CANNAME);

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
  private TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
  private CANdiConfiguration candiConfiguration = new CANdiConfiguration();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private NeutralOut neutralOut = new NeutralOut();

  private double offset = 0.0;

  public PivotIOKrakenX60() {
    pivotConfiguration.MotorOutput.withNeutralMode(
        PivotConstants.PIVOT_NEUTRAL_MODE_COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    pivotConfiguration.MotorOutput.withInverted(
        PivotConstants.PIVOT_INVERSION
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive);
    pivotConfiguration.Slot0.kP = PivotConstants.PIVOT_KP;
    pivotConfiguration.Slot0.kI = PivotConstants.PIVOT_KI;
    pivotConfiguration.Slot0.kD = PivotConstants.PIVOT_KD;
    pivotConfiguration.Slot0.kA = PivotConstants.PIVOT_KA;
    pivotConfiguration.Slot0.kS = PivotConstants.PIVOT_KS;
    pivotConfiguration.Slot0.kV = PivotConstants.PIVOT_KV;
    pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        PivotConstants.PIVOT_MOTION_MAGIC_CRUISE_VELOCITY;
    pivotConfiguration.MotionMagic.MotionMagicAcceleration =
        PivotConstants.PIVOT_MOTION_MAGIC_ACCELERATION;
    pivotConfiguration.MotorOutput.withPeakForwardDutyCycle(
        PivotConstants.PIVOT_FORWARD_DUTY_CYCLE_LIMIT);
    pivotConfiguration.MotorOutput.withPeakReverseDutyCycle(
        PivotConstants.PIVOT_REVERSE_DUTY_CYCLE_LIMIT);
    pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable =
        PivotConstants.PIVOT_FORWARD_SOFT_LIMIT_ENABLE;
    pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable =
        PivotConstants.PIVOT_REVERSE_SOFT_LIMIT_ENABLE;
    pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD;
    pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD;
    pivotConfiguration.Feedback.FeedbackRemoteSensorID = PivotConstants.CANDI_ID;
    pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANdiPWM1;
    pivotConfiguration.Feedback.SensorToMechanismRatio =
        PivotConstants.PIVOT_CANDI_2_MECHANISM_RATIO;
    pivotConfiguration.Feedback.RotorToSensorRatio = PivotConstants.PIVOT_ROTOR_2_CANDI_RATIO;
    pivot.getConfigurator().apply(pivotConfiguration);

    candiConfiguration.PWM1.SensorDirection = PivotConstants.CANDI_DIRECTION;
    candiConfiguration.PWM1.AbsoluteSensorOffset = PivotConstants.CANDI_OFFSET;
    candi.getConfigurator().apply(candiConfiguration);

    offset = candi.getPWM1Position().getValueAsDouble();

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
                PivotConstants.PIVOT_SIGNAL_UPDATE_FREQUENCY_HZ,
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

  private double getPositionwithoutOffset() {
    return candi.getPWM1Position().getValueAsDouble() - offset;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, tempCelsius).isOK();
    inputs.position = position.getValueAsDouble();
    inputs.positionWithoutOffset = getPositionwithoutOffset();
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
    pivot.setControl(
        voltageOut
            .withOutput(volts)
            .withLimitForwardMotion(
                getPositionwithoutOffset() > PivotConstants.PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD)
            .withLimitReverseMotion(
                getPositionwithoutOffset() < PivotConstants.PIVOT_REVERSE_SOFT_LIMIT_THRESHOLD));
  }

  @Override
  public void setAngle(double position) {
    pivot.setControl(
        motionMagic
            .withPosition(position + offset)
            .withLimitForwardMotion(
                getPositionwithoutOffset() > PivotConstants.PIVOT_FORWARD_SOFT_LIMIT_THRESHOLD)
            .withLimitReverseMotion(
                getPositionwithoutOffset() < PivotConstants.PIVOT_REVERSE_SOFT_LIMIT_THRESHOLD));
  }

  @Override
  public void hold() {
    setAngle(this.getPositionwithoutOffset());
  }

  @Override
  public void stop() {
    pivot.setControl(neutralOut);
  }

  @Override
  public boolean isAtPosition(double position) {
    return Math.abs(this.getPositionwithoutOffset() - position)
        < PivotConstants.PIVOT_POSITION_TOLERANCE;
  }
}
