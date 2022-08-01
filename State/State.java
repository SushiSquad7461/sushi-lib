package SushiFrcLib.State;

public class State {
    public static enum SolenoidState {
        EXTEND(true),
        RETRACT(false);

        private final boolean state;

        private SolenoidState(boolean state) {
            this.state = state;
        }

        public boolean get() {
            return state;
        }
    }

    public static enum SuperiorState {
        ASSESS,
        DISABLED,
        MANUAL_CONTROL,
        ENABLED;
    }
}
