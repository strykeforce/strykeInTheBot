package frc.robot.subsystems.shoulder;

import frc.robot.constants.MinimalShoulderConstants;
import java.util.Set;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.TelemetryService;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

public class MinimalShoulderSubsystem extends MeasurableSubsystem {
    public static final double kShelfMove = 0;
    private final MinimalShoulderIO io;
    private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private ShoulderStates currState = ShoulderStates.IDLE;
    private ShoulderStates desiredState = ShoulderStates.IDLE;
    private double setPointTicks = inputs.positionTicks;
    private Logger logger = LoggerFactory.getLogger(ShoulderSubsystem.class);
    private org.littletonrobotics.junction.Logger advLogger = org.littletonrobotics.junction.Logger.getInstance();
    private int zeroStableCounts = 0;

    public MinimalShoulderSubsystem(MinimalShoulderIO io) {
        this.io = io;
    }

    public ShoulderStates getState() {
        return currState;
    }

    public void stow() {
        io.setPos(MinimalShoulderConstants.kStowShoulderPos);
        desiredState = ShoulderStates.STOW;
        currState = ShoulderStates.TRANSITION;
    }


    public void zero() {
        io.configSoftLimitEnable(false);
        io.configStatorCurrentLimit(MinimalShoulderConstants.getElevStatorCurrentLimitConfiguration());
        io.setPct(MinimalShoulderConstants.kZeroSpeed);
        logger.info("Minimal Shoulder is zeroing");
        currState = ShoulderStates.ZEROING;
    }

    public void setPos(double position) {
        io.setSelectedSensorPos(position);
        setPointTicks = position;
    }

    public boolean isFinished() {
        if (currState == ShoulderStates.TRANSITION) {
            switch (desiredState) {
                case IDLE:
                    return true;
                default:
                    return Math.abs(setPointTicks - inputs.positionTicks) <= MinimalShoulderConstants.kCloseEnough;
            }
        } else {
            return true;
        }
    }

    public boolean isPastPoint(double pastPointTicks) {

        // Tests if the pos is between the input and the SetPoint.
        // This is the best way I could find.

        if (setPointTicks < inputs.positionTicks) {
            return (pastPointTicks + MinimalShoulderConstants.kCloseEnough >= inputs.positionTicks);
        } else if (setPointTicks > inputs.positionTicks) {
            return (pastPointTicks - MinimalShoulderConstants.kCloseEnough <= inputs.positionTicks);
        } else {
            return true;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        advLogger.processInputs("Shoulder", inputs);

        switch (currState) {
            case IDLE:
                break;

            case ZEROING:
                if (Math.abs(io.getSelectedSensorPosition()) < MinimalShoulderConstants.kZeroTargetSpeedTicksPer100ms) {
                    zeroStableCounts++;
                } else {
                    zeroStableCounts = 0;
                }

                if (zeroStableCounts > MinimalShoulderConstants.kZeroStableCounts) {
                    io.setPct(0);
                    io.setSelectedSensorPos(0.0);

                    io.setPct(0);
                    io.configStatorCurrentLimit(MinimalShoulderConstants.getShoulderStatorTurnOff());
    
                    io.configSupplyCurrentLimit(
                            MinimalShoulderConstants.getShoulderSupplyLimitConfig(),
                            MinimalShoulderConstants.kTalonConfigTimeout);

                    io.configSoftLimitEnable(true);

                    setPointTicks = 0;
                    currState = ShoulderStates.ZEROED;
                    logger.info("Shoulder is zeroed");
                }
                break;

            case TRANSITION:
                if (isFinished()) {
                    logger.info("{} -> {}", currState, desiredState);
                    currState = desiredState;
                }
                break;

            default:
        }

        // Log Outputs
        advLogger.recordOutput("Shoulder/currState", currState.ordinal());
        advLogger.recordOutput("Shoulder/setpointTicks", setPointTicks);
    }

    // States
    public enum ShoulderStates {
        IDLE,
        TRANSITION,
        STOW,
        ZEROING,
        ZEROED
    }

    @Override
    public Set<Measure> getMeasures() {
        return Set.of(new Measure("State", () -> getState().ordinal()));
    }

    @Override
    public void registerWith(TelemetryService telemetryService) {
        super.registerWith(telemetryService);
        io.registerWith(telemetryService);
    }
}
