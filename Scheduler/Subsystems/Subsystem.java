package SushiFrcLib.Scheduler.Subsystems;

import SushiFrcLib.CheesyLibUtil.LatchedBoolean;
import SushiFrcLib.Scheduler.Loops.Loop.Phase;
import SushiFrcLib.State.State.SuperiorState;
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

    
    private SuperiorState superiorState = SuperiorState.ENABLED;
    private SuperiorState prevSuperiorState = superiorState;
    private boolean manualControl = false;

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


    public final int whenRunAgain () { return desiredPeriod; }

    public void passInIndex(int listIndex){
        mListIndex = listIndex;
    }

    public final void onStart(Phase phase) {
        switch (phase) {
            case TEST:
                superiorState = SuperiorState.ASSESS;
                break;
            case DISABLED:
                superiorState = SuperiorState.DISABLED;
            default:
                superiorState = SuperiorState.ENABLED;
        }

        start(phase);
        mStateChanged = true;
        System.out.println(subsystemName + " state " + getCurrentState());
        mLB_SystemStateChange.update(false); // reset
        onStop(); // put into a known state
    }

    public abstract void start(Phase phase);

    public final void onLoop(double timestamp) {
        synchronized (this) {
            double now = Timer.getFPGATimestamp();
            actualPeriod = now - lastSchedStart;
            lastSchedStart = now;

            transferState();

            switch (superiorState) {
                case ENABLED:
                    periodic();
                    break;
                case DISABLED:
                    break;
                case ASSESS:
                    assessPeriodic();
                case MANUAL_CONTROL:
                    manualControlPeriodic();
            }
        }
    }

    public void turnOnManulControl(String caller) {
        prevSuperiorState = superiorState;
        superiorState = SuperiorState.MANUAL_CONTROL;
        manualControl = true;
        System.out.println("Manual control turned on by " + caller);
    }

    public void turnOffManulControl(String caller) {
        if (manualControl) {
            superiorState = prevSuperiorState;
            manualControl = false;
            System.out.println("Manual control turned off by " + caller);
        } else {
            System.out.println("Manual control was trying to be turned off but alread off by " + caller);
        }
    }

    // Run during test mode
    public void assessPeriodic() { System.out.println("Assess Periodic for " + subsystemName + " has not been provided"); } 

    public void manualControlPeriodic() {  System.out.println("Manual Control Periodic for " + subsystemName + " has not been provided"); }

    protected boolean stateChanged() {
        if(mStateChanged) {
            mStateChanged = false;
            return true;
        }
        return false;
    }

    public abstract void periodic();

    public void zeroSensors() {}
    
    public final void onStop() {
        stop();
    }

    public abstract void stop();

    public final String getLogHeaders() {
        String headers = getPeriodicLogHeaders();

        return subsystemName+".schedDeltaDesired,"+
        subsystemName +".schedDeltaActual,"+
        subsystemName+".schedDuration," + 
        subsystemName+".mSystemState" + (headers.length() > 0 ? "," + headers : "");
    }

    public String getPeriodicLogHeaders() { return ""; }

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

    public String getLogValues() { return ""; }


    public void outputTelemetry() {}

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

    protected void setInitalState(SubsystemState newState) {
        currentState = newState;
        wantedState = newState;
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