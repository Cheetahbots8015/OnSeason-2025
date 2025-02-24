package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ElevatorConstants;
import frc.robot.util.MagicTimer;

public class ElevatorSubsystem extends SubsystemBase {
	protected final ElevatorSystemIOInputsAutoLogged inputs = new ElevatorSystemIOInputsAutoLogged();
	private final String name;
	private final ElevatorSystemIO io;
	private final Alert disconnected;

	public ElevatorSubsystem(String name, ElevatorSystemIO io) {
		this.name = name;
		this.io = io;
		disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
	}

	public boolean isAtPosition(double height) {
		return io.isAtPosition(height);
	}

	public boolean isAbovePosition(double height) {
		return this.getLeaderPositionWithoutOffset() > height;
	}

	public void home() {
		io.setHome();
	}

	public void L1() {
		setPosition_MotionMagicianLow(ElevatorConstants.L1Position);
	}

	public void L2() {
		setPosition_MotionMagicianLow(ElevatorConstants.L2Position);
	}

	public void L3() {
		setPosition_MotionMagicianLow(ElevatorConstants.L3Position);
	}

	public void L4() {
		setPosition_MotionMagicianLow(ElevatorConstants.L4Position);
	}

	public void lowAlgae() {
		setPosition_MotionMagician(ElevatorConstants.lowAlgaePosition);
	}

	public void highAlgae() {
		setPosition_MotionMagician(ElevatorConstants.highAlgaePosition);
	}

	public boolean getHallSensorActive() {
		return !hallSensor.get();
	}

	public void report() {
		SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
		SmartDashboard.putNumber("elevator/leader position", leader.getPosition().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/follower position", follower.getPosition().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/position difference",
				leader.getPosition().getValueAsDouble() - follower.getPosition().getValueAsDouble());
		SmartDashboard.putBoolean("elevator/hallsensor", getHallSensorActive());
		SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
		SmartDashboard.putNumber("elevator/leader encoder offset", leaderEncoderOffset);
		SmartDashboard.putNumber("elevator/follower encoder offset", followerEncoderOffset);
		SmartDashboard.putNumber("elevator/leader velocity", leader.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/leader acceleration", leader.getAcceleration().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/leader torquecurrent", leader.getTorqueCurrent().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/follower torquecurrent", follower.getTorqueCurrent().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/motionmagic target", leader.getClosedLoopReference().getValueAsDouble());
		SmartDashboard.putNumber("elevator/leaderVoltage", leader.getMotorVoltage().getValueAsDouble());
		SmartDashboard.putNumber(
				"elevator/followerVoltage", follower.getMotorVoltage().getValueAsDouble());
	}

	// at the beginning of homing process, the phase should be phase1
	// if the elevator is in phase1 and hall sensor is active, then switch to phase2
	// otherwise, if the elevator is in phase2 and hall sensor is not active, the
	// switch to phase3
	// elevator in phase2 will move up and the move down until hall sensor is active
	// again
	// elevator in phase3 will directly move down until hall sensor is active
	// by the end of homing process, the phase should be set back to phase1
	private enum homePhase {
		phase1,
		phase2,
		phase3
	}
}
