package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(51, "canivore");
    private final TalonFX follower = new TalonFX(52, "canivore");
    TalonFXConfiguration leaderconfigs = new TalonFXConfiguration();
    TalonFXConfiguration followerconfigs = new TalonFXConfiguration();

    public ElevatorSubsystem() {
        leader.getConfigurator().apply(leaderconfigs);
        follower.getConfigurator().apply(followerconfigs);
        follower.setControl(new Follower(51, true));
    }

    public void ShutDown() {
        leader.setControl(new NeutralOut());
    }

    public void RunVolts(double duty){
        
        leader.setControl(new DutyCycleOut(duty));
        
        
    }

    public void report(){
        SmartDashboard.putNumber("leader position", leader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("position difference", leader.getPosition().getValueAsDouble()-follower.getPosition().getValueAsDouble());
    }

}
