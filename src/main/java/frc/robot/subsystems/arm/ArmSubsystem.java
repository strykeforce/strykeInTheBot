package frc.robot.subsystems.arm;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class ArmSubsystem extends MeasurableSubsystem {
    private ArmStates armState = ArmStates.STOW;

    public ArmSubsystem() {}

    @Override
    public Set<Measure> getMeasures() {
        // TODO Auto-generated method stub
        return null;
    }

    public enum ArmStates {
        STOW,
        FLOOR,
        SHELF_CUBE,
        SHELF_CONE,
        SCORE,
        TO_STOW,
        TO_FLOOR,
        TO_SHELF_CUBE,
        TO_SHELF_CONE,
        TO_SCORE,
    }
}
