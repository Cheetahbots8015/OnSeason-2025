package frc.robot.subsystems.pivot;

import static frc.robot.generated.PivotConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.util.LoggedTunableNumber;

public class PivotIOKrakenX60 implements PivotIO {

  /* Hardware */
  private final TalonFX pivot = new TalonFX(PIVOT_ID);
  private final CANdi candi = new CANdi(CANDI_ID);

  /* Configurators */
  private TalonFXConfigurator pivotConfigurator;
  private CANdiConfigurator candiConfigurator;

  private final CurrentLimitsConfigs currentLimitConfigs;
  private final MotorOutputConfigs motorOutputConfigs;
  private final Slot0Configs slot0Configs;
  private final MotionMagicConfigs pivotMotionMagicConfigs;

  /* Gains */
  LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", PIVOT_KS);
  LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/kA", PIVOT_KA);
  LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", PIVOT_KV);
  LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", PIVOT_KP);
  LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI", PIVOT_KI);
  LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", PIVOT_KD);

  LoggedTunableNumber motionAcceleration =
      new LoggedTunableNumber("Pivot/MotionAcceleration", PIVOT_MOTION_MAGIC_ACCELERATION);
  LoggedTunableNumber motionCruiseVelocity =
      new LoggedTunableNumber("Pivot/MotionCruiseVelocity", PIVOT_MOTION_MAGIC_ACCELERATION);

  public PivotIOKrakenX60() {
    /* Configurators */
    pivotConfigurator = pivot.getConfigurator();
    candiConfigurator = candi.getConfigurator();

    /* Configure pivot hardware */
    currentLimitConfigs = new CurrentLimitsConfigs();

    motorOutputConfigs = new MotorOutputConfigs();

    slot0Configs = new Slot0Configs();
    slot0Configs.kS = kS.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kV = kV.get();
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();

    pivotMotionMagicConfigs = new MotionMagicConfigs();

    OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();

    ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();

    /* Apply Configurations */
    pivotConfigurator.apply(currentLimitConfigs);
    pivotConfigurator.apply(motorOutputConfigs);
    pivotConfigurator.apply(slot0Configs);
    pivotConfigurator.apply(pivotFeedbackConfigs);
    pivotConfigurator.apply(pivotMotionMagicConfigs);
    pivotConfigurator.apply(openLoopRampsConfigs);
    pivotConfigurator.apply(closedLoopRampsConfigs);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {}

  @Override
  public void updateTunableNumbers() {
    if (kS.hasChanged(0)
        || kA.hasChanged(0)
        || kV.hasChanged(0)
        || kP.hasChanged(0)
        || kI.hasChanged(0)
        || kD.hasChanged(0)
        || motionAcceleration.hasChanged(0)
        || motionCruiseVelocity.hasChanged(0)) {
      slot0Configs.kS = kS.get();
      slot0Configs.kA = kA.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();

      pivotMotionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
      pivotMotionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();

      pivotConfigurator.apply(slot0Configs);
      pivotConfigurator.apply(pivotMotionMagicConfigs);
    }
  }
}
