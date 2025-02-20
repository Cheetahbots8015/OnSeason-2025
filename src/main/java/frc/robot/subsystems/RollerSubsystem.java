package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RollerConstants;

public class RollerSubsystem extends SubsystemBase {
  private final TalonFX roller = new TalonFX(RollerConstants.rollerID, RollerConstants.canName);
  private final CANrange canRange =
      new CANrange(RollerConstants.canRangeID, RollerConstants.canName);
  private final CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();

  private TalonFXConfiguration rollerconfigs = new TalonFXConfiguration();

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private VelocityTorqueCurrentFOC velocityFOC = new VelocityTorqueCurrentFOC(0.0);
  private NeutralOut neutralOut = new NeutralOut();

  private double timer = -1.0;

  public RollerSubsystem() {
    rollerconfigs.MotorOutput.withInverted(
        RollerConstants.inverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    rollerconfigs.Slot0.kP = RollerConstants.kP;
    rollerconfigs.Slot0.kI = RollerConstants.kI;
    rollerconfigs.Slot0.kD = RollerConstants.kD;
    rollerconfigs.Slot0.kA = RollerConstants.kA;
    rollerconfigs.Slot0.kS = RollerConstants.kS;
    rollerconfigs.Slot0.kV = RollerConstants.kV;

    canRangeConfigs.ProximityParams.ProximityThreshold = RollerConstants.canRangeThreshold;
    canRangeConfigs.ProximityParams.MinSignalStrengthForValidMeasurement =
        RollerConstants.minSignalStrength;
    canRangeConfigs.ProximityParams.ProximityHysteresis = RollerConstants.canRangeHysteresis;

    canRange.getConfigurator().apply(canRangeConfigs);

    roller.getConfigurator().apply(rollerconfigs);
  }

  public void shutDown() {
    roller.setControl(neutralOut);
  }

  public void manualForwardVolts() {
    roller.setControl(voltageOut.withOutput(RollerConstants.manualForwardVoltage));
  }

  public void manualReverseVolts() {
    roller.setControl(voltageOut.withOutput(RollerConstants.manualReverseVoltage));
  }

  public void station() {
    roller.setControl(voltageOut.withOutput(RollerConstants.stationVoltage));
  }

  public void L1Vots() {
    roller.setControl(voltageOut.withOutput(RollerConstants.L1Voltage));
  }

  public void L2Vots() {
    roller.setControl(voltageOut.withOutput(RollerConstants.L2Voltage));
  }

  public void L3Vots() {
    roller.setControl(voltageOut.withOutput(RollerConstants.L3Voltage));
  }

  public void L4Vots() {
    roller.setControl(voltageOut.withOutput(RollerConstants.L4Voltage));
  }

  public void AlgaeVolts() {
    roller.setControl(voltageOut.withOutput(RollerConstants.AlgaeVoltage));
  }

  public void setVelocity(double velocity) {
    roller.setControl(velocityFOC.withVelocity(velocity));
  }

  public boolean intakeFinished() {
    return canRange.getIsDetected().getValue();
  }

  public void stopIntaking() {
    if (timer == -1.0) {
      timer = Timer.getFPGATimestamp();
    }

    if (Timer.getFPGATimestamp() - timer < RollerConstants.stopIntakingTime) {
      station();
    } else {
      shutDown();
    }
  }

  public void resetTimer() {
    timer = -1.0;
  }

  public void report() {
    SmartDashboard.putNumber("roller/velocity", roller.getVelocity().getValueAsDouble());
  }
}
