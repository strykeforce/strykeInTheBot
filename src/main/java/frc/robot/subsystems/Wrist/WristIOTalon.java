package frc.robot.subsystems.Wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.constants.WristConstants;
import org.strykeforce.telemetry.TelemetryService;

public class WristIOTalon implements WristIO {

  private TalonFX Talon;

  public WristIOTalon() {
    Talon = new TalonFX(WristConstants.kWristTalonMainId);
    Talon.configFactoryDefault();
    Talon.configAllSettings(WristConstants.getWristTalonConfig());
    Talon.configSupplyCurrentLimit(WristConstants.getWristSupplyLimitConfig());
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
  public void updateInputs(WristIOInputs inputs) {
    inputs.positionTicks = Talon.getSelectedSensorPosition();
    inputs.absoluteTicks = Talon.getSensorCollection().getIntegratedSensorAbsolutePosition();
    inputs.velocityTicksPer100ms = Talon.getSelectedSensorVelocity();
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(Talon);
  }
}
