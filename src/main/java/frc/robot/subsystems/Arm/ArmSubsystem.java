package frc.robot.subsystems.Arm;

import java.util.Set;

import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import frc.robot.subsystems.Extendo.ExtendoSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;

public class ArmSubsystem extends MeasurableSubsystem {

    private final ShoulderSubsystem shoulderSubsystem;
    private final ExtendoSubsystem extendoSubsystem;
    private final WristSubsystem wristSubsystem;
        

    public ArmSubsystem (ShoulderSubsystem shoulderSubsystem, ExtendoSubsystem extendoSubsystem, WristSubsystem wristSubsystem) {
        this.shoulderSubsystem = shoulderSubsystem;
        this.extendoSubsystem = extendoSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    public void stow () {
        
    }

    public enum ArmStates {
        SHELF_CONE,
        SHELF_CUBE,
        FLOOR_CONE,
        FLOOR_CONE_UPRIGHT,
        FLOOR_CUBE,
        HIGH_CUBE,
        HIGH_CONE,
        MID_CUBE,
        MID_CONE,
        LOW_CUBE,
        LOW_CONE,
        STOW,
        PARALLEL_TRANS
    }

    @Override
    public Set<Measure> getMeasures() {
        // TODO Fill in measures for grapher
        return null;
    }
    
}
