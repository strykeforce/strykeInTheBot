package frc.robot.subsystems.drive;
import org.littletonrobotics.junction.AutoLog;
import org.strykeforce.telemetry.TelemetryService;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public interface ModulesIO {
    
    @AutoLog
    public static class ModulesIOInputs {

    }

    public default void updateInputs(ModulesIOInputs inputs) {}

    public default void setSelectedSensorPos(double positionTicks) {}
  
    public default void setPos(double positionTicks) {}
  
    public default void setPct(double percentOutput) {}
  
    public default void setSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration) {}
  
    public default void registerWith(TelemetryService telemetryService) {}


}
