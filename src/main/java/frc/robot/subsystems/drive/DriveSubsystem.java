package frc.robot.subsystems.drive;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Set;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.telemetry.measurable.MeasurableSubsystem;
import org.strykeforce.telemetry.measurable.Measure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import net.consensys.cava.toml.Toml;
import net.consensys.cava.toml.TomlArray;
import net.consensys.cava.toml.TomlParseResult;
import net.consensys.cava.toml.TomlTable;

public class DriveSubsystem extends MeasurableSubsystem {
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private static final Logger logger = LoggerFactory.getLogger(DriveSubsystem.class);
    private org.littletonrobotics.junction.Logger advLogger = org.littletonrobotics.junction.Logger.getInstance();
    private final Swerve swerve;

    public DriveSubsystem(Swerve swerve) {
        this.swerve = swerve;
    }   

    

    public Trajectory generateTrajectory(String trajectoryName){
        Trajectory trajectoryGenerated = new Trajectory();
        try{
            TomlParseResult parseResult = Toml.parse(Paths.get("/home/lvuser/deploy/paths/" + trajectoryName + ".toml"));
            logger.info("Generating Trajectory: {}", trajectoryName);
            Pose2d startPose = new Pose2d(
                parseResult.getTable("start_pose").getDouble("x"),
                parseResult.getTable("start_pose").getDouble("y"),
                Rotation2d.fromDegrees(parseResult.getTable("start_pose").getDouble("angle"))
            );
            Pose2d endPose = new Pose2d(
                parseResult.getTable("end_pose").getDouble("x"),
                parseResult.getTable("end_pose").getDouble("y"),
                Rotation2d.fromDegrees(parseResult.getTable("end_pose").getDouble("angle"))
            );
            TomlArray internalPointsToml = parseResult.getArray("internal_points");
            ArrayList<Translation2d> path = new ArrayList<>();
            logger.info("Toml Internal Points Array Size: {}", internalPointsToml.size());

            for(int i = 0; i < internalPointsToml.size(); i++){
                TomlTable waypointToml = internalPointsToml.getTable(i);
                Translation2d waypoint = new Translation2d(
                    waypointToml.getDouble("x"),
                    waypointToml.getDouble("y")
                );
                path.add(waypoint);
            }
            
            //Trajectory Config parsed from toml - any additional constraints would be added here
            TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                parseResult.getDouble("max_velocity"),
                parseResult.getDouble("max_acceleration")
            );
            trajectoryConfig.setReversed(parseResult.getBoolean("is_reversed"));
            trajectoryConfig.setStartVelocity(parseResult.getDouble("start_velocity"));
            trajectoryConfig.setEndVelocity(parseResult.getDouble("end_velocity"));

            trajectoryGenerated = TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);
        }catch (IOException error){
            logger.error(error.toString());
            logger.error("Path {} not found", trajectoryName);
        }
        return trajectoryGenerated;
    }


    @Override
    public Set<Measure> getMeasures() {
        // TODO Auto-generated method stub
        return null;
    }
}
