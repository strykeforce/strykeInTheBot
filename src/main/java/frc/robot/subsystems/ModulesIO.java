
import org.littletonrobotics.junction.AutoLog;

public interface ModulesIO {
    
    @AutoLog
    public static class ModulesIOInputs {
        public double odometryX = 0.0;
        public double odometryY = 0.0;
    }

}
