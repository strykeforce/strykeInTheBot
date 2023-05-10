package frc.robot.subsystems.hand;

import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class HandSubsystem extends MeasurableSubsystem {
    private final HandIO io;
    private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();
    private HandStates currState = HandStates.IDLE;
    private HandStates desiredState = HandStates.IDLE;
    private Logger logger = LoggerFactory.getLogger(HandSubsystem.class);
    private org.littletonrobotics.junction.Logger advLogger = org.littletonrobotics.junction.Logger.getInstance();

    public HandSubsystem(HandIO io) {
        this.io = io;
    }

    public HandStates getState() {
        return currState;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        advLogger.processInputs("Hand", inputs);

        switch (currState) {
            case IDLE:
                break;
            case CONE:
                break;
            case CUBE:
                break;
            case WAITING:
                break;
        }
    }

    public enum HandStates {
        IDLE,
        CONE,
        CUBE,
        WAITING
    }

    public Set<Measure> getMeasures() {
        return Set.of(new Measure("State", () -> getState().ordinal()));
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        super.registerWith(telemetryService);
        io.registerWith(telemetryService);
    }
}
