package SushiFrcLib.Constants;

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
}