package frc.robot.subsystems.hand;


import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.constants.HandConstants;

public class HandIOFalcon implements HandIO {
    
    private TalonFX hand;

    public HandIOFalcon() {
        hand = new TalonFX(HandConstants.kHandFalconID);
        hand.configFactoryDefault();
        hand.configAllSettings(HandConstants.getHandFalconConfig());
        hand.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void setPct(double percentOutput) {
        hand.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setSupplyCurrentLimit(SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {
        hand.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    }

    @Override
    public void updateInputs(HandIOInputs inputs) {
        inputs.velocityTicksPer100ms = hand.getSelectedSensorVelocity();
        inputs.isFwdLimitSwitchClosed = hand.isFwdLimitSwitchClosed() == 1.0;
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        telemetryService.register(hand);
    }
}