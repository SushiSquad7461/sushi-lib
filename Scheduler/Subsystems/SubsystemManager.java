package SushiFrcLib.Scheduler.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

import SushiFrcLib.Scheduler.Loops.ILooper;
import SushiFrcLib.Scheduler.Loops.Loop;
import SushiFrcLib.Scheduler.Loops.Looper;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
    private List<Subsystem> mAllSubsystems;
    private SubsystemLogManager mSSLogMngr;
    private int mSSCount;
    private TheLoop mTheLoop;
    private String[] mSSEmptyLog;
    private int mLoopPeriod;

    private static String sClassName;
    private static int sInstanceCount;
    private static SubsystemManager sInstance = null;
    public  static SubsystemManager getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new SubsystemManager(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private SubsystemManager(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
    }

    public void initializeSubsystemManager(int loopPeriod, List<Subsystem> allSubsystems) {
        mLoopPeriod = loopPeriod;
        mAllSubsystems = allSubsystems;
        mSSCount = mAllSubsystems.size();
        mTheLoop = new TheLoop();
        mSSLogMngr = new SubsystemLogManager();
        mSSEmptyLog = new String[mSSCount];
    }

    // enabling and disabling the logger is handled in the logger
    // to simplify the rest of the code it assumes logging is always enabled
    private void prepAllSubsystemsForLogging(boolean disableLogging) {
        mSSLogMngr.startLogging(disableLogging);
        int i=0;
        mSSLogMngr.addToLine("LineNum,LoopNum,TicToc,Time,Schedule");
        for (Subsystem s : mAllSubsystems) {
            String header = s.getLogHeaders();
            // count commas
            int columns = header.length() - header.replace(",", "").length();
            // create string of just commas
            mSSEmptyLog[i++] = ",".repeat(columns);
            mSSLogMngr.addToLine(header);
        }

        mSSLogMngr.endLine();
    }

    private int runALoop(int index){
        Subsystem s = mAllSubsystems.get(index);
        s.onLoop(Timer.getFPGATimestamp());
        mSSLogMngr.addToLine(s.getLogValues(false));
        return s.whenRunAgain();
    }

    // subsystems call these two methods when they want to wake up or run soon
    public void scheduleMe(int listIndex, int when){
        scheduleMe(listIndex, when, true);
    }

    public void scheduleMe(int listIndex, int when, boolean clearSchedule){
        mTheLoop.scheduleMe(listIndex, when, clearSchedule);
    }

    private class TheLoop implements Loop {
        private final int lInitialDelay = 10;
        private long[] lSchedule = new long[mSSCount];
        private long lTicToc;
        private int lLineNum;
        private Phase lPhase;
        private long lLostTime;
    
        @Override
        public void onStart(Phase thePhase) {
            // thePhase is bogus, get the real one below
            boolean disableLogging = false;

            lPhase = getPhase();
            for (Subsystem s : mAllSubsystems) {
                s.onStart(lPhase);
            }

            // Set time to be invalid
            for (int i=0; i<lSchedule.length; i++){
                lSchedule[i] = -1;
            }
            
            // get current time and turn it into MSec's
            lTicToc = (long)(Timer.getFPGATimestamp()*1000.0+.5); // System.currentTimeMillis();

            // schedule all subsystems to start, spread out
            for (int i=0; i<mSSCount; i++){
                lSchedule[i] = (long)(lTicToc+lInitialDelay+i);
            }

            lLineNum=0;

            // when disabled a log file is not wanted
            if (lPhase == Phase.DISABLED){
                disableLogging = true;
            }

            prepAllSubsystemsForLogging(disableLogging);
       }

        @Override
        public void onLoop(double timestamp) {
            // each time this is called it loops mLoopPeriod times
            // mLoopPeriod is the number msecs between calls
            // one loop per msec
            for (int i=0; i<mLoopPeriod; i++){
                // Stringfy current schedule, TODO: figure out how to optimize
                String schedule = "";
                for (int j=0; j < lSchedule.length-1; j++) {
                    schedule += lSchedule[j] + ":";
                }
                schedule += lSchedule[lSchedule.length-1];

                mSSLogMngr.addToLine(""+lLineNum++ +","+i+","+lTicToc+","+Timer.getFPGATimestamp()+","+schedule);

                for (int j=0; j<lSchedule.length; j++){
                    // Final part of if statment to help prevent errors, assumes that scheudler will never be 50ms behind
                    if (lTicToc >= lSchedule[j] && lSchedule[j] >= 0 && (lTicToc - lSchedule[j] < 50)) {
                        // run all phases for this SS and get next time to run in delta MSec
                        int nextRun = runALoop(j);

                        // 0 means do not schedule
                        if (nextRun != 0){
                            lSchedule[j] = lTicToc+nextRun;
                        } else {
                            lSchedule[j] = -1;
                        }
                    } else {
                        mSSLogMngr.addToLine(mSSEmptyLog[j]);
                    }
                }

                mSSLogMngr.endLine();

                // advance to next schedule slot
                lTicToc++;

                // every 250 msec handle telemetry, but no longer log
                if (lTicToc%250 == 0){
                    // mSSLogMngr.addToLine(""+lLineNum++ +",,"+lTicToc+","+Timer.getFPGATimestamp()+",-1");
                    for (Subsystem s : mAllSubsystems) {
                        // mSSLogMngr.addToLine(s.getLogValues(true));
                        s.outputTelemetry();
                    }
                    // mSSLogMngr.endLine();
                }

                // these lines are only needed if mLoopPeriod is greater than the actual period
                // currently they are equal
                // if mLoopPeriod is greater then this stops the loop from getting ahead
                lLostTime = ((long)(Timer.getFPGATimestamp()*1000.0+.5))-lTicToc;
                if (lLostTime <= 0){
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.onStop();
            }
            System.out.println("schedule time lost "+lLostTime);
        }

        public void scheduleMe(int listIndex, int when, boolean clearScheduled){
            if (listIndex < mSSCount) {
                lSchedule[listIndex] = lTicToc+when;
            }
        }

        // This could be passed in
        private Phase getPhase(){
            if(DriverStation.isEnabled()){
                if (DriverStation.isTeleop()){
                    return Phase.TELEOP;
                }
                else if (DriverStation.isAutonomous()){
                    return Phase.AUTONOMOUS;
                }
                else if (DriverStation.isTest()){
                    return Phase.TEST;
                }
            }
            return Phase.DISABLED;
        }
    }

    // ask all subsystems to register themselves then register with the looper
    public void registerEnabledLoops(Looper looper2) {
        int index = 0;

        for (Subsystem s : mAllSubsystems) {
            s.passInIndex(index);
            index += 1;
        }
        looper2.register(mTheLoop);
    }

    // unused
    @Override
    public void start(){}

    // unused
    @Override
    public void outputToSmartDashboard() {}
 
    // unused
    @Override
    public void stop() {}

    @Override
    public int register(Loop loop) {
        return 0;
    }
 }
