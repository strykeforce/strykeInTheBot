package frc.robot.subsystems.shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ShoulderConstants;
import org.strykeforce.telemetry.TelemetryService;

public class ShoulderTalonIO implements ShoulderIO {
  private TalonSRX shoulderLeftMain;
  private TalonSRX shoulderRightFollow;

  public ShoulderTalonIO() {
    shoulderLeftMain = new TalonSRX(ShoulderConstants.kLeftMainID);
    configTalon(shoulderLeftMain);
    shoulderLeftMain.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 20);
    shoulderRightFollow = new TalonSRX(ShoulderConstants.kLeftFollowID);
    configTalon(shoulderRightFollow);

    shoulderRightFollow.follow(shoulderLeftMain);
  }

  private void configTalon(TalonSRX falcon) {
    falcon.configFactoryDefault();
    falcon.configAllSettings(ShoulderConstants.getShoulderTalonConfig());
    falcon.configSupplyCurrentLimit(ShoulderConstants.getShoulderSupplyLimitConfig());
    falcon.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void setSelectedSensorPos(double positionTicks) {
    shoulderLeftMain.setSelectedSensorPosition(positionTicks);
  }

  @Override
  public void setPos(double positionTicks) {
    shoulderLeftMain.set(ControlMode.MotionMagic, positionTicks);
  }

  @Override
  public void setPct(double percentOutput) {
    shoulderRightFollow.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    shoulderLeftMain.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    shoulderRightFollow.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }

  @Override
  public void disableOutput() {
    shoulderLeftMain.configPeakOutputForward(0);
    shoulderLeftMain.configPeakOutputReverse(0);
    shoulderRightFollow.configPeakOutputForward(0);
    shoulderRightFollow.configPeakOutputReverse(0);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.positionTicks = shoulderLeftMain.getSelectedSensorPosition();
    inputs.absoluteTicks = shoulderLeftMain.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    inputs.velocityTicksPer100ms = shoulderLeftMain.getSelectedSensorVelocity();
    inputs.isFwdLimitSwitchClosed = shoulderLeftMain.isFwdLimitSwitchClosed() == 1.0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shoulderLeftMain);
    telemetryService.register(shoulderRightFollow);
  }
}
