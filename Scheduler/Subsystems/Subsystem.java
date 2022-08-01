package SushiFrcLib.Scheduler.Subsystems;

import SushiFrcLib.ChesyLibUtil.LatchedBoolean;
import SushiFrcLib.Scheduler.Loops.Loop.Phase;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public abstract class Subsystem<SubsystemState extends Enum<SubsystemState>>  {
    private SubsystemManager mSubsystemManager;

    // Id of subsystem
    protected int mListIndex;

    // States
    private SubsystemState currentState;
    protected SubsystemState wantedState;

    protected String subsystemName;
    private int sInstanceCount = 0;
    private boolean mStateChanged;
    private LatchedBoolean mLB_SystemStateChange = new LatchedBoolean();

    public int desiredPeriod = 20; // in ms, set to 20 by defualt

    public double actualPeriod;
    public double lastSchedStart;


    public Subsystem() {
        this.subsystemName = this.getClass().getSimpleName();
        mSubsystemManager = SubsystemManager.getInstance(subsystemName);
    }

    protected void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + subsystemName + " " + ++sInstanceCount);
    }

    protected boolean setPeriod(int newPeriod) {
        if (newPeriod > 0) {
            desiredPeriod = newPeriod;
            return true;
        }
        System.out.println("Somebody tried setting the period of " + subsystemName + " to " + newPeriod + "ms");
        return false;
    }


    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public final void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        actualPeriod = now - lastSchedStart;
        lastSchedStart = now;

        readPeriodic();
    }

    public void readPeriodic() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public final void writePeriodicOutputs() {
        writePeriodic();
    }

    public void writePeriodic() {}


    public final int whenRunAgain () { return desiredPeriod; }

    public void passInIndex(int listIndex){
        mListIndex = listIndex;
    }

    public final void onStart(Phase phase) {
        start(phase);
        mStateChanged = true;
            System.out.println(subsystemName + " state " + getCurrentState());
        mLB_SystemStateChange.update(false); // reset
            onStop(); // put into a known state
    }

    public abstract void start(Phase phase);


    public final void onLoop(double timestamp){
        synchronized (this) {
            do {
                changeState();
                if (wantedState != currentState) {
                    System.out.println(
                        subsystemName + " state " + currentState + " to " + wantedState + " (" + timestamp + ")");
                    currentState = wantedState;
                    mStateChanged = true;
                } else {
                }
            } while (mLB_SystemStateChange.update(mStateChanged));
        }
    }

    protected boolean stateChanged() {
        if(mStateChanged) {
            mStateChanged = false;
            return true;
        }
        return false;
    }

    public abstract void changeState();

    public void zeroSensors() {}
    
    public final void onStop() {
        stop();
        writePeriodicOutputs();
    }

    public abstract void stop();

    public final String getLogHeaders() {
        String headers = getPeriodicLogHeaders();

        return subsystemName+".schedDeltaDesired,"+
        subsystemName +".schedDeltaActual,"+
        subsystemName+".schedDuration," + 
        subsystemName+".mSystemState" + (headers.length() > 0 ? "," + headers : "");
    }

    public abstract String getPeriodicLogHeaders();


    public final String getLogValues(boolean telemetry) {
        String ret;
        if (telemetry){
            ret = ",,,";
        }
        else{
            ret = desiredPeriod + "," +
            actualPeriod+","+
                    (Timer.getFPGATimestamp()-lastSchedStart)+",";
        }
        String vals = getLogValues();

        return ret + getCurrentState() + (vals.length() > 0 ?  "," + vals : "");
    }

    public abstract String getLogValues();


    public abstract void outputTelemetry();

    public void scheduleForStateChange() {
        mSubsystemManager.scheduleMe(mListIndex, 1, true);
    }

    public synchronized SubsystemState getCurrentState() {
        return currentState;
    }

    protected SubsystemState transferState() {
        currentState = wantedState;
        return currentState;
    }

    protected void setState(SubsystemState newState) {
        currentState = newState;
    }


    // this method should only be used by external subsystems.
    // if you want to change your own wantedState then simply set
    // it directly
    public synchronized void setWantedState(SubsystemState state, String who) {
        if (state != wantedState) {
            wantedState = state;
            scheduleForStateChange();
            System.out.println(who + " is setting wanted state of " + subsystemName + " to " + state);
        } else {
            //System.out.println(who + " is setting wanted state of " + subsystemName + " to " + state + " again!!!");
        }
    }

    public SubsystemState getWantedState() {
        return wantedState;
    }
}