package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public interface HandIO {
    
    @AutoLog
    public static class HandIOInputs {
        public double velocityTicksPer100ms = 0.0;
        public boolean isFwdLimitSwitchClosed = false;
    }

    public default void updateInputs(HandIOInputs inputs) {}

    public default void setPct(double percentOutput) {}

    public default void setSupplyCurrentLimit(SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}

    public default void registerWith(TelemetryService telemetryService) {}
}
