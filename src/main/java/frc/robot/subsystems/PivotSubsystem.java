package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Generated.ElevatorConstants;
import frc.robot.Generated.PivotConstants;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFX pivot = new TalonFX(PivotConstants.pivotID, PivotConstants.canName);

    private TalonFXConfiguration pivotconfigs = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private NeutralOut neutralOut = new NeutralOut();

    private double encoderOffset = 0.0;

    private final SysIdRoutine m_SysIdRoutinePivot = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(1.5),
                    Seconds.of(3.0),
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        pivot.setVoltage(volts.magnitude() * 0.5);
                    },
                    null,
                    this));

    private final SysIdRoutine routineToApply = m_SysIdRoutinePivot;

    public PivotSubsystem() {
        pivotconfigs.MotorOutput.withInverted(PivotConstants.inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
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
        pivotconfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotconfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotconfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 16.5;
        pivotconfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.3;
        pivot.getConfigurator().apply(pivotconfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(250, pivot.getPosition(), pivot.getVelocity(),
                pivot.getMotorVoltage());

        SignalLogger.start();
    }

    public void shutDown() {
        pivot.setControl(neutralOut);
    }

    public void manualVoltsForward() {
        report();
        pivot.setControl(voltageOut.withOutput(PivotConstants.manualVoltageForward));
    }

    public void manualVoltsReverse() {
        report();
        pivot.setControl(voltageOut.withOutput(PivotConstants.manualVoltageReverse));
    }

    public void setHeight(double height) {
        report();
        pivot.setControl(motionMagic.withPosition(height + encoderOffset));
    }

    public void set2L2() {
        setHeight(PivotConstants.L2Position);
    }

    public void set2L4() {
        setHeight(PivotConstants.L4Position);
    }

    public void home() {
        setHeight(0.0);
    }

    public void hold() {
        pivot.setControl(motionMagic.withPosition(pivot.getPosition().getValueAsDouble() - encoderOffset));
    }

    public void resetOffset() {
        encoderOffset = pivot.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double height) {
        height += encoderOffset;
        return Math.abs(pivot.getPosition().getValueAsDouble() - height) < PivotConstants.positionDeadband;
    }

    public void report() {
        SmartDashboard.putNumber("pivot/pivot position", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("pivot/encoder offset", encoderOffset);
        SmartDashboard.putNumber("pivot/pivot velocity", pivot.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("pivot/pivot acceleration", pivot.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("pivot/pivot torquecurrent", pivot.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("pivot/motionmagic target", pivot.getClosedLoopReference().getValueAsDouble());
    }

    public Command PivotTestDynamic(SysIdRoutine.Direction direction) {
        return routineToApply.dynamic(direction);
    }

    public Command PivotTestQuasistatic(SysIdRoutine.Direction direction) {
        return routineToApply.quasistatic(direction);
    }

}
