package frc.robot.subsystems.Extendo;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.ExtendoConstants;
import org.strykeforce.telemetry.TelemetryService;

public class ExtendoIOTalon implements ExtendoIO {

  private TalonFX Talon;
  private TalonFX Talon2;

  public ExtendoIOTalon() {
    Talon = new TalonFX(ExtendoConstants.kExtendoTalonMainId);
    Talon2 = new TalonFX(ExtendoConstants.kExtendoTalonFollowId);
    Talon.configFactoryDefault();
    Talon.configAllSettings(ExtendoConstants.getExtendoTalonConfig());
    Talon.configSupplyCurrentLimit(ExtendoConstants.getExtendoSupplyLimitConfig());
    Talon.setNeutralMode(NeutralMode.Brake);
    Talon2.configFactoryDefault();
    Talon2.configAllSettings(ExtendoConstants.getExtendoTalonConfig());
    Talon2.configSupplyCurrentLimit(ExtendoConstants.getExtendoSupplyLimitConfig());
    Talon2.setNeutralMode(NeutralMode.Brake);
    Talon2.follow(Talon);
  }

  @Override
  public void setSelectedSensorPos(double positionTicks) {
    Talon.setSelectedSensorPosition(positionTicks);
  }

  @Override
  public void setPos(double positionTicks) {
    Talon.set(ControlMode.MotionMagic, positionTicks);
  }

  @Override
  public void setPct(double percentOutput) {
    Talon.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    Talon.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }

  @Override
  public void updateInputs(ExtendoIOInputs inputs) {
    inputs.positionTicks = Talon.getSelectedSensorPosition();
    inputs.absoluteTicks = Talon.getSensorCollection().getIntegratedSensorAbsolutePosition();
    inputs.velocityTicksPer100ms = Talon.getSelectedSensorVelocity();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(Talon);
  }
}
