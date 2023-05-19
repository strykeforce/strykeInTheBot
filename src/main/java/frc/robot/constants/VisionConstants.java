package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static Matrix<N3, N1> kStateStdDevs =
    VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

    // Increase these numbers to trust sensor readings from encoders and gyros less. This matrix is
    // in the form [theta], with units in radians.
    public static Matrix<N1, N1> kLocalMeasurementStdDevs =
        VecBuilder.fill(Units.degreesToRadians(0.01));

    // Increase these numbers to trust global measurements from vision less. This matrix is in the
    // form [x, y, theta]ᵀ, with units in meters and radians.
    // Vision Odometry Standard devs
    public static Matrix<N3, N1> kVisionMeasurementStdDevs =
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
}
