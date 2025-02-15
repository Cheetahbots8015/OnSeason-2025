package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Generated.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(ElevatorConstants.leaderID, ElevatorConstants.canName);
    private final TalonFX follower = new TalonFX(ElevatorConstants.followerID, ElevatorConstants.canName);
    private DigitalInput hallSensor = new DigitalInput(ElevatorConstants.hallSensorID);

    private TalonFXConfiguration leaderconfigs = new TalonFXConfiguration();
    private TalonFXConfiguration followerconfigs = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0.0);
    private NeutralOut neutralOut = new NeutralOut();

    public ElevatorSubsystem() {
        leaderconfigs.MotorOutput.withInverted(ElevatorConstants.inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        leaderconfigs.Slot0.kP = ElevatorConstants.kP;
        leaderconfigs.Slot0.kI = ElevatorConstants.kI;
        leaderconfigs.Slot0.kD = ElevatorConstants.kD;
        leaderconfigs.Slot0.kA = ElevatorConstants.kA;
        leaderconfigs.Slot0.kS = ElevatorConstants.kS;
        leaderconfigs.Slot0.kV = ElevatorConstants.kV;
        leaderconfigs.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.forwardDutyCycleLimit);
        leaderconfigs.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.reverseDutyCycleLimit);
        leader.getConfigurator().apply(leaderconfigs);
        follower.getConfigurator().apply(followerconfigs);
        follower.setControl(new Follower(ElevatorConstants.leaderID, ElevatorConstants.opposeMaster));
    }

    public void shutDown() {
        leader.setControl(neutralOut);
    }

    public void manualVolts() {
        leader.setControl(voltageOut.withOutput(ElevatorConstants.manualVoltage));
    }

    public void lockVolts(){
        leader.setControl(voltageOut.withOutput(ElevatorConstants.lockVoltage));
    }

    public void setHeight(double height) {
        leader.setControl(motionMagic.withPosition(height));
    }

    public boolean getHallSensorActive() {
        return !hallSensor.get();
    }

    public void report() {
        SmartDashboard.putNumber("elevator/leader position", leader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator/position difference",
                leader.getPosition().getValueAsDouble() - follower.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("elevator/hallsensor", getHallSensorActive());
    }

}
