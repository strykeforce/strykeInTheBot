package frc.robot.subsystems.shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.MinimalShoulderConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;

public class MinimalShoulderFalconSimIO implements MinimalShoulderIO {
  private TalonFX shoulder;
  private Logger logger = LoggerFactory.getLogger(MinimalShoulderIO.class);
  private final TalonFXSimState simFX = shoulder.getSimState();
  private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 11.88, 0.369, 0.41, 0.0,
      Math.PI / 2.0, true);

  public MinimalShoulderFalconSimIO() {
    shoulder = new TalonFX(MinimalShoulderConstants.kLeftMainID);
    configFalcon(shoulder);

    armSim.setInput(shoulder.getSupplyVoltage().getValue());
  }

  private void configFalcon(TalonFX falcon) {
    falcon.getConfigurator().apply(new TalonFXConfiguration());
    falcon.getConfigurator().apply(MinimalShoulderConstants.getShoulderFalconP6Config());
    falcon.getConfigurator().apply(MinimalShoulderConstants.getShoulderZeroCurrentSim());
  }

  public void configSoftLimitEnable(boolean enable) {
    HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
    config.ForwardLimitEnable = enable;
    config.ReverseLimitEnable = enable;
    shoulder.getConfigurator().apply(config);
  }

  public void configStatorCurrentLimit(
      CurrentLimitsConfigs statorCurrentLimitConfiguration) {
    shoulder.getConfigurator().apply(statorCurrentLimitConfiguration);
  }

  @Override
  public void setSelectedSensorPos(double positionTicks) {
    shoulder.setRotorPosition(positionTicks);
  }

  @Override
  public void setPos(double positionTicks) {
    shoulder.setControl(new MotionMagicDutyCycle(positionTicks));
    logger.info("Set Shoulder Pos: {}", positionTicks);
  }

  @Override
  public void setPct(double percentOutput) {
    shoulder.setControl(new DutyCycleOut(percentOutput));
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
      CurrentLimitsConfigs config = new CurrentLimitsConfigs();
      config.SupplyCurrentLimit = supplyCurrentLimitConfiguration.currentLimit;
      config.SupplyCurrentLimitEnable = supplyCurrentLimitConfiguration.enable;
      shoulder.getConfigurator().apply(config);
  }

  @Override
  public void disableOutput() {
    shoulder.disable();
  }

  @Override
  public void updateInputs(MinimalShoulderIOInputs inputs) {
    armSim.update(0.02);
    inputs.positionTicks = shoulder.getPosition().getValue();
    inputs.absoluteTicks = shoulder.getRotorPosition().getValue();
    inputs.velocityTicksPer100ms = shoulder.getVelocity().getValue();
    inputs.isFwdLimitSwitchClosed = shoulder.getForwardLimit().getValue().value == 1.0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
  }

  @Override
  public double getSelectedSensorPosition() {
    return shoulder.getRotorPosition().getValue();
  }

  @Override
  public double getSelectedSensorVelocity() {
    return shoulder.getRotorVelocity().getValue();
  }

  @Override
  public void configSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration shoulderSupplyLimitConfig, int ktalonconfigtimeout) {
      CurrentLimitsConfigs config = new CurrentLimitsConfigs();
      config.SupplyCurrentLimit = shoulderSupplyLimitConfig.currentLimit;
      config.SupplyCurrentLimitEnable = shoulderSupplyLimitConfig.enable; 
      config.SupplyCurrentThreshold = shoulderSupplyLimitConfig.triggerThresholdCurrent;
      config.SupplyTimeThreshold = shoulderSupplyLimitConfig.triggerThresholdTime;
    shoulder.getConfigurator().apply(config, ktalonconfigtimeout);
  }

  @Override
  public void configStatorCurrentLimit(
    StatorCurrentLimitConfiguration elevStatorCurrentLimitConfiguration) {
      CurrentLimitsConfigs config = new CurrentLimitsConfigs();
      config.SupplyCurrentLimit = elevStatorCurrentLimitConfiguration.currentLimit;
      config.SupplyCurrentLimitEnable = elevStatorCurrentLimitConfiguration.enable; 
      config.SupplyCurrentThreshold = elevStatorCurrentLimitConfiguration.triggerThresholdCurrent;
      config.SupplyTimeThreshold = elevStatorCurrentLimitConfiguration.triggerThresholdTime;
    shoulder.getConfigurator().apply(config);
  }
}
