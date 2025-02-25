package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

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
		return inputs.positionWithoutOffset[0] > height;
	}

	public void home() {
		io.setHome();
	}

	public void L1() {
		io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L1_POSITION);
	}

	public void L2() {
		io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L2_POSITION);
	}

	public void L3() {
		io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L3_POSITION);
	}

	public void L4() {
		io.setPosition_MotionMagicianLow(ElevatorConstants.ELEVATOR_L4_POSITION);
	}

	public void lowAlgae() {
		io.setPosition_MotionMagician(ElevatorConstants.ELEVATOR_LOW_ALGAE_POSITION);
	}

	public void highAlgae() {
		io.setPosition_MotionMagician(ElevatorConstants.ELEVATOR_HIGH_ALGAE_POSITION);
	}

	public void defaultDown() {
		io.defaultFall();
	}

	public void resetFilter() {
		io.resetFilter();
	}

	public void homeInit(){
		io.resetFilter();
		io.resetHomePhase();
	}
}
