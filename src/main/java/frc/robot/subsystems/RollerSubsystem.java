package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Generated.RollerConstants;

public class RollerSubsystem extends SubsystemBase {
    private final TalonFX roller = new TalonFX(RollerConstants.rollerID, RollerConstants.canName);
    private final CANrange canRange = new CANrange(RollerConstants.canRangeID, RollerConstants.canName);

    private TalonFXConfiguration rollerconfigs = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private VelocityTorqueCurrentFOC velocityFOC = new VelocityTorqueCurrentFOC(0.0);
    private NeutralOut neutralOut = new NeutralOut();

    public RollerSubsystem() {
        rollerconfigs.MotorOutput.withInverted(
                RollerConstants.inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);
        rollerconfigs.Slot0.kP = RollerConstants.kP;
        rollerconfigs.Slot0.kI = RollerConstants.kI;
        rollerconfigs.Slot0.kD = RollerConstants.kD;
        rollerconfigs.Slot0.kA = RollerConstants.kA;
        rollerconfigs.Slot0.kS = RollerConstants.kS;
        rollerconfigs.Slot0.kV = RollerConstants.kV;
        roller.getConfigurator().apply(rollerconfigs);
    }

    public void shutDown() {
        roller.setControl(neutralOut);
    }

    public void runVolts() {
        roller.setControl(voltageOut.withOutput(RollerConstants.manualVoltage));
    }

    public void L1Vots (){
        roller.setControl(voltageOut.withOutput(RollerConstants.lowerVoltage));
    }

    public void L2Vots() {
        roller.setControl(voltageOut.withOutput(RollerConstants.manualVoltage));
    }

    public void L3Vots() {
        roller.setControl(voltageOut.withOutput(RollerConstants.manualVoltage));
    }

    public void L4Vots() {
        roller.setControl(voltageOut.withOutput(RollerConstants.manualVoltage));
    }

    public void setVelocity(double velocity) {
        roller.setControl(velocityFOC.withVelocity(velocity));
    }

    public void report() {
        SmartDashboard.putNumber("roller/velocity", roller.getVelocity().getValueAsDouble());
    }

}
