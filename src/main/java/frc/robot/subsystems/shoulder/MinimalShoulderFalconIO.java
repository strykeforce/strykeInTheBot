package frc.robot.subsystems.shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.MinimalShoulderConstants;
import org.strykeforce.telemetry.TelemetryService;

public class MinimalShoulderFalconIO implements MinimalShoulderIO {
  private TalonFX shoulder;

  public MinimalShoulderFalconIO() {
    shoulder = new TalonFX(MinimalShoulderConstants.kLeftMainID);
    configFalcon(shoulder);
    shoulder.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 20);
  }

  private void configFalcon(TalonFX falcon) {
    falcon.configFactoryDefault();
    falcon.configAllSettings(MinimalShoulderConstants.getShoulderFalconConfig());
    falcon.configSupplyCurrentLimit(MinimalShoulderConstants.getShoulderSupplyLimitConfig());
    falcon.setNeutralMode(NeutralMode.Brake);
  }

  public void configSoftLimitEnable(boolean enable) {
    shoulder.configForwardSoftLimitEnable(enable);
    shoulder.configReverseSoftLimitEnable(enable);
  }

  public void configStatorCurrentLimit(StatorCurrentLimitConfiguration statorCurrentLimitConfiguration) {
    shoulder.configStatorCurrentLimit(statorCurrentLimitConfiguration);
  }

  @Override
  public void setSelectedSensorPos(double positionTicks) {
    shoulder.setSelectedSensorPosition(positionTicks);
  }

  @Override
  public void setPos(double positionTicks) {
    shoulder.set(ControlMode.MotionMagic, positionTicks);
  }

  @Override
  public void setPct(double percentOutput) {
    shoulder.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    shoulder.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }

  @Override
  public void disableOutput() {
    shoulder.configPeakOutputForward(0);
    shoulder.configPeakOutputReverse(0);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.positionTicks = shoulder.getSelectedSensorPosition();
    inputs.absoluteTicks = shoulder.getSensorCollection().getIntegratedSensorAbsolutePosition();
    inputs.velocityTicksPer100ms = shoulder.getSelectedSensorVelocity();
    inputs.isFwdLimitSwitchClosed = shoulder.isFwdLimitSwitchClosed() == 1.0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shoulder);
  }

  @Override
  public double getSelectedSensorPosition() {
    return shoulder.getSelectedSensorPosition();
  }

  @Override
  public void configSupplyCurrentLimit(SupplyCurrentLimitConfiguration shoulderSupplyLimitConfig,
      int ktalonconfigtimeout) {
    shoulder.configSupplyCurrentLimit(shoulderSupplyLimitConfig, ktalonconfigtimeout);

  }
}
