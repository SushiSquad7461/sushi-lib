package SushiFrcLib.Constants;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;

public class SushiConstants {
    public static class OI {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 0;
    }
    public static class SCHEDULER {
        public static final double LOOPPERIOD = .005;
        public static final double LOOPPERIODMS = LOOPPERIOD * 1000;
    }
    public static class DEPENDENCY_INJECTION {
        public static final String FILE_PATH = "/home/lvuser/name.txt";
        public static final String COMP_NAME = "comp";
    }
    public static class SWERVE_MODULE {
        public static final double ERROR_BOUND = 2;
        public static final double TURN_SPEED = 0.9;
    }
    public static class Ports {
        public static final int PIGEON = 13;
    }
    public static class SDS_SWERVE {
        public static final Mk4SwerveModuleHelper.GearRatio GEAR_RATIO = Mk4SwerveModuleHelper.GearRatio.L2;
        public static final Mk4iSwerveModuleHelper.GearRatio FOUR_I_GEAR_RATIO = Mk4iSwerveModuleHelper.GearRatio.L2;
    }
}
