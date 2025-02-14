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

public class RollerSubsystem extends SubsystemBase {
    private final TalonFX roller = new TalonFX(62, "canivore");
    TalonFXConfiguration rollerconfigs = new TalonFXConfiguration();

    public RollerSubsystem() {
        rollerconfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        roller.getConfigurator().apply(rollerconfigs);
    }

    public void ShutDown() {
        roller.setControl(new NeutralOut());
    }

    public void RunVolts(double duty){
        
        roller.setControl(new DutyCycleOut(duty));
        
        
    }

    public void report(){
       
    }

}
