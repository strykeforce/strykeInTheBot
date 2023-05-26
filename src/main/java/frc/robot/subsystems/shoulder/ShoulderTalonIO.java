package frc.robot.subsystems.shoulder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.ShoulderConstants;
import org.strykeforce.telemetry.TelemetryService;

public class ShoulderTalonIO implements ShoulderIO {
  private TalonSRX shoulderLeft1Main;
  private TalonSRX shoulderLeft2Follow;
 
  

  public ShoulderTalonIO() {
    shoulderLeft1Main = new TalonSRX(ShoulderConstants.kLeftMainID);
    configTalon(shoulderLeft1Main);
    shoulderLeft2Follow = new TalonSRX(ShoulderConstants.kLeftFollowID);
    configTalon(shoulderLeft2Follow);
    
    shoulderLeft2Follow.follow(shoulderLeft1Main);
    
  }

  private void configTalon(TalonSRX falcon)
  {
    falcon.configFactoryDefault();
    falcon.configAllSettings(ShoulderConstants.getShoulderTalonConfig());
    falcon.configSupplyCurrentLimit(ShoulderConstants.getShoulderSupplyLimitConfig());
    falcon.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void setSelectedSensorPos(double positionTicks) {
    shoulderLeft1Main.setSelectedSensorPosition(positionTicks);
    
  }

  @Override
  public void setPos(double positionTicks) {
    shoulderLeft1Main.set(ControlMode.MotionMagic, positionTicks);
  }

  @Override
  public void setPct(double percentOutput) {
    shoulderLeft2Follow.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    shoulderLeft1Main.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    shoulderLeft1Main.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.positionTicks = shoulderLeft1Main.getSelectedSensorPosition();
    inputs.absoluteTicks = shoulderLeft1Main.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    inputs.velocityTicksPer100ms = shoulderLeft1Main.getSelectedSensorVelocity();
    inputs.isFwdLimitSwitchClosed = shoulderLeft1Main.isFwdLimitSwitchClosed() == 1.0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(shoulderLeft1Main);
    telemetryService.register(shoulderLeft2Follow);
    
  }
}
