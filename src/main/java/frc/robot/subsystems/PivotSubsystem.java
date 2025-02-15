package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Generated.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    private final TalonFX pivot = new TalonFX(PivotConstants.pivotID, PivotConstants.canName);

    private TalonFXConfiguration pivotconfigs = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0.0);
    private NeutralOut neutralOut = new NeutralOut();

    private double encoderOffset = 0.0;

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
        pivot.getConfigurator().apply(pivotconfigs);
    }

    public void shutDown() {
        pivot.setControl(neutralOut);
    }

    public void manualVoltsForward() {
        report();
        pivot.setControl(voltageOut.withOutput(PivotConstants.manualVoltageForward));
    }

    public void manualVoltsReverse(){
        report();
        pivot.setControl(voltageOut.withOutput(PivotConstants.manualVoltageReverse));
    }

    public void setHeight(double height) {
        report();
        pivot.setControl(motionMagic.withPosition(height+encoderOffset));
    }

    public void set2L2(){
        setHeight(PivotConstants.L2Position);
    }

    public void hold(){
        pivot.setControl(motionMagic.withPosition(pivot.getPosition().getValueAsDouble()-encoderOffset));
    }

    public void resetOffset(){
        encoderOffset = pivot.getPosition().getValueAsDouble();
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

}
