package frc.robot.subsystems.example;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.ExampleConstants;
import org.strykeforce.telemetry.TelemetryService;

public class ExampleIOTalon implements ExampleIO {

  private TalonSRX example;

  public ExampleIOTalon() {
    example = new TalonSRX(ExampleConstants.kExampleTalonId);
    example.configFactoryDefault();
    example.configAllSettings(ExampleConstants.getExampleTalonConfig());
    example.configSupplyCurrentLimit(ExampleConstants.getExampleSupplyLimitConfig());
    example.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void setSelectedSensorPos(double positionTicks) {
    example.setSelectedSensorPosition(positionTicks);
  }

  @Override
  public void setPos(double positionTicks) {
    example.set(ControlMode.MotionMagic, positionTicks);
  }

  @Override
  public void setPct(double percentOutput) {
    example.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void setSupplyCurrentLimit(
      SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
    example.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
  }

  @Override
  public void updateInputs(ExampleIOInputs inputs) {
    inputs.positionTicks = example.getSelectedSensorPosition();
    inputs.absoluteTicks = example.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    inputs.velocityTicksPer100ms = example.getSelectedSensorVelocity();
    inputs.isFwdLimitSwitchClosed = example.isFwdLimitSwitchClosed() == 1.0;
  }

  @Override
  public void registerWith(TelemetryService telemetryService) {
    telemetryService.register(example);
  }
}
