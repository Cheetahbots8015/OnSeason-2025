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
			io.setAngle(PivotConstants.ALGAE);
		} else {
			io.setAngle(PivotConstants.coralHomePosition);
		}
	}

	public void L1() {
		io.setAngle(PivotConstants.L1Position);
	}

	public void L2() {
		io.setAngle(PivotConstants.L2Position);
	}

	public void L3() {
		io.setAngle(PivotConstants.L3Position);
	}

	public void L4() {
		io.setAngle(PivotConstants.L4Position);
	}

	public void lowAlgae() {
		io.setAngle(PivotConstants.lowAlgaePosition);
	}

	public void highAlgae() {
		io.setAngle(PivotConstants.highAlgaePosition);
	}

	public void processor() {
		io.setAngle(PivotConstants.processorPosition);
	}

	public void hold() {
		io.setAngle(this.getPositionwithoutOffset());
	}

	public boolean isAtPosition(double height) {
		if (Math.abs(this.getPositionwithoutOffset() - height) < PivotConstants.positionDeadband) {
			SmartDashboard.putBoolean("pivot/is at position", true);
		} else {
			SmartDashboard.putBoolean("pivot/is at position", false);
		}
		return Math.abs(this.getPositionwithoutOffset() - height) < PivotConstants.positionDeadband;
	}

	public void report() {
		SmartDashboard.putNumber("pivot/pivot position", pivot.getPosition().getValueAsDouble());
		SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
		SmartDashboard.putNumber("pivot/pivot velocity", pivot.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber(
				"pivot/pivot acceleration", pivot.getAcceleration().getValueAsDouble());
		SmartDashboard.putNumber(
				"pivot/pivot torquecurrent", pivot.getTorqueCurrent().getValueAsDouble());
		SmartDashboard.putNumber(
				"pivot/motionmagic target", pivot.getClosedLoopReference().getValueAsDouble());
		SmartDashboard.putNumber("pivot/s1 position", candi.getPWM1Position().getValueAsDouble());
		SmartDashboard.putNumber("pivot/offset", offset);
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
