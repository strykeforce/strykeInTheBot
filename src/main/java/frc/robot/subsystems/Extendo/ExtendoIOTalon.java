package frc.robot.subsystems.Extendo;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ExtendoConstants;
import org.strykeforce.telemetry.TelemetryService;

public class ExtendoIOTalon implements ExtendoIO {

  private TalonSRX Talon;

  public ExtendoIOTalon() {
    Talon = new TalonSRX(ExtendoConstants.kExtendoTalonId);
    Talon.configFactoryDefault();
    Talon.configAllSettings(ExtendoConstants.getExtendoTalonConfig());
    Talon.configSupplyCurrentLimit(ExtendoConstants.getExtendoSupplyLimitConfig());
    Talon.setNeutralMode(NeutralMode.Brake);
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
    inputs.absoluteTicks = Talon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    inputs.velocityTicksPer100ms = Talon.getSelectedSensorVelocity();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(Talon);
  }
}
