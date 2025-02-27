package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RollerConstants;
import frc.robot.util.MagicTimer;

public class RollerSubsystem extends SubsystemBase {
  // krakenX60 initialization with configs
  private final TalonFX roller = new TalonFX(RollerConstants.rollerID, RollerConstants.canName);
  private TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

  // canRange initialization with configs
  private final CANrange canRange =
      new CANrange(RollerConstants.canRangeID, RollerConstants.canName);
  private final CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();

  // control methods initialization
  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private VelocityTorqueCurrentFOC velocityFOC = new VelocityTorqueCurrentFOC(0.0);
  private NeutralOut neutralOut = new NeutralOut();

  // magic timer initialization
  private MagicTimer rollerTimer = new MagicTimer();

  // indicate whether holding algae, defaultly regarded as holding coral since the
  // idle velocity of
  // holding coral is zero
  public enum rollerIdleState {
    coral,
    algae,
    manual
  }

  private rollerIdleState systemIdleState = rollerIdleState.coral;

  public RollerSubsystem() {
    // config neutralmode
    rollerConfigs.MotorOutput.withNeutralMode(
        RollerConstants.neutalmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    // config direction
    // POSITIVE for intaking and placing the corals
    rollerConfigs.MotorOutput.withInverted(
        RollerConstants.inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    // config PIDSAV
    rollerConfigs.Slot0.kP = RollerConstants.kP;
    rollerConfigs.Slot0.kI = RollerConstants.kI;
    rollerConfigs.Slot0.kD = RollerConstants.kD;
    rollerConfigs.Slot0.kA = RollerConstants.kA;
    rollerConfigs.Slot0.kS = RollerConstants.kS;
    rollerConfigs.Slot0.kV = RollerConstants.kV;

    // apply roller configs
    roller.getConfigurator().apply(rollerConfigs);

    // config canRange
    canRangeConfigs.ProximityParams.ProximityThreshold = RollerConstants.canRangeThreshold;
    canRangeConfigs.ProximityParams.MinSignalStrengthForValidMeasurement =
        RollerConstants.minSignalStrength;
    canRangeConfigs.ProximityParams.ProximityHysteresis = RollerConstants.canRangeHysteresis;

    // apply canRange configs
    canRange.getConfigurator().apply(canRangeConfigs);
  }

  // shutDown krankenX60
  public void shutDown() {
    roller.setControl(neutralOut);
  }

  // basic function to run voltage out, should be used by multiple functions
  public void setVolts(double volts) {
    report();
    roller.setControl(voltageOut.withOutput(volts));
  }

  // basic function to run torque current velocity, should be used by multiple
  // functions
  public void setVelocity(double velocity) {
    report();
    roller.setControl(velocityFOC.withVelocity(velocity));
  }

  // for operator and test
  public void manualForwardVolts() {
    setVolts(RollerConstants.manualForwardVoltage);
  }

  // for operator and test
  public void manualReverseVolts() {
    setVolts(RollerConstants.manualReverseVoltage);
  }

  // for test
  public void manualForwardVelocity() {
    setVelocity(RollerConstants.manualForwardVelocity);
  }

  // for test
  public void manualReverseVelocity() {
    setVelocity(RollerConstants.manualReverseVelocity);
  }

  // for default command
  public void defaultIdelVelocity() {
    if (systemIdleState == rollerIdleState.algae) {
      setVelocity(RollerConstants.algaeIdelVelocity);
    } else if (systemIdleState == rollerIdleState.coral) {
      setVelocity(RollerConstants.coralIdleVelocity);
    } else {
      shutDown();
    }
  }

  // for driver, spinning the roller until a certain time has passed since the
  // canRange detected
  // coral;
  public void station() {
    if (isCanRangeActive()) {
      rollerTimer.startTimer();
      if (rollerTimer.getTimePassedSec() > RollerConstants.stationTime) {
        setVelocity(RollerConstants.coralIdleVelocity);
      } else {
        setVelocity(RollerConstants.stationVelocity);
      }
    } else {
      rollerTimer.resetTimer();
      setVelocity(RollerConstants.stationVelocity);
    }
  }

  public void L1() {
    setVelocity(RollerConstants.L1Velocity);
  }

  public void L2() {
    setVelocity(RollerConstants.L2Velocity);
  }

  public void L3() {
    setVelocity(RollerConstants.L3Velocity);
  }

  public void L4() {
    setVelocity(RollerConstants.L4Velocity);
  }

  public void HoldAlgaeVolts() {
    report();
    roller.setControl(voltageOut.withOutput(-3.0));
  }

  public void intakeAlgaefromReef() {
    setVelocity(RollerConstants.intakeAlgaefromReefVelocity);
  }

  public void processor() {
    setVelocity(RollerConstants.processorVelocity);
  }

  public boolean isCanRangeActive() {
    canRange.getIsDetected().refresh();
    return canRange.getIsDetected().getValue();
  }

  public void report() {
    SmartDashboard.putNumber("roller/velocity", roller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("roller/torque current", roller.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber("roller/acceleration", roller.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("roller/supply voltage", roller.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("roller/canrange", isCanRangeActive());
    SmartDashboard.putString("roller/idle state", systemIdleState.toString());
  }

  // methods to manage isHoldAlgae boolean
  // methods to manage system idle state
  public rollerIdleState getSysteIdleState() {
    return systemIdleState;
  }

  public void setSystemIdleState(rollerIdleState state) {
    systemIdleState = state;
  }

  public void switchIdleState() {
    if (systemIdleState == rollerIdleState.algae) {
      systemIdleState = rollerIdleState.coral;
    } else if (systemIdleState == rollerIdleState.coral) {
      systemIdleState = rollerIdleState.algae;
    } else {
      systemIdleState = rollerIdleState.coral;
    }
  }
}
