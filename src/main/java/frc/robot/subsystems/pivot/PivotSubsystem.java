package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
	private final PivotIO io;
	private final String name;
	private final Alert disconnected;
	private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

	private final SysIdRoutine sysIdRoutinePivot;
	private final SysIdRoutine routineToApply;

	private boolean isHoldAlgae = false;
	public PivotSubsystem(String name, PivotIO pivotIO) {
		this.name = name;
		this.io = pivotIO;
		disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);

		this.sysIdRoutinePivot =
				new SysIdRoutine(
						new SysIdRoutine.Config(
								null,
								Volts.of(1.5),
								Seconds.of(3.5),
								(state) -> SignalLogger.writeString("state", state.toString())),
						new SysIdRoutine.Mechanism(
								(volts) -> {
									io.setVoltage(volts.magnitude() * 0.5);
								},
								null,
								this));
		this.routineToApply = sysIdRoutinePivot;
	}

	public void manualVoltsForward() {
		io.setVoltage(PivotConstants.PIVOT_MANUAL_FORWARD_VOLTAGE);
	}

	// for operator and test
	public void manualVoltsReverse() {
		io.setVoltage(PivotConstants.PIVOT_MANUAL_REVERSE_VOLTAGE);
	}

	public void idle() {
		if (isHoldAlgae) {
			io.setAngle(PivotConstants.PIVOT_ALGAE_HOLD_ANGLE);
		} else {
			io.setAngle(PivotConstants.PIVOT_CORAL_HOLD_ANGLE);
		}
	}

	public void L1() {
		io.setAngle(PivotConstants.PIVOT_L1_ANGLE);
	}

	public void L2() {
		io.setAngle(PivotConstants.PIVOT_L2_ANGLE);
	}

	public void L3() {
		io.setAngle(PivotConstants.PIVOT_L3_ANGLE);
	}

	public void L4() {
		io.setAngle(PivotConstants.PIVOT_L4_ANGLE);
	}

	public void lowAlgae() {
		io.setAngle(PivotConstants.PIVOT_LOWALGAE_ANGLE);
	}

	public void highAlgae() {
		io.setAngle(PivotConstants.PIVOT_HIGHALGAE_ANGLE);
	}

	public void processor() {
		io.setAngle(PivotConstants.PIVOT_PROCESSOR_ANGLE);
	}

	public void hold() {
		io.setAngle(inputs.positionWithoutOffset);
	}

	public boolean isAtPosition(double height) {
		return io.isAtPosition(height);
	}


	// methods to manage isHoldAlgae boolean
	public boolean getHoldAlgae() {
		return isHoldAlgae;
	}

	public void setHoldAlgae(boolean set) {
		isHoldAlgae = set;
	}

	public void switchHoldAlgae() {
		isHoldAlgae = !isHoldAlgae;
	}

	// sysID command
	public Command PivotTestDynamic(SysIdRoutine.Direction direction) {
		return routineToApply.dynamic(direction);
	}

	// sysID command
	public Command PivotTestQuasistatic(SysIdRoutine.Direction direction) {
		return routineToApply.quasistatic(direction);
	}
}
