package SushiFrcLib.State;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

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

    public static enum MotorNeturalState {
        COAST(true),
        BRAKE(false);


        private final boolean coast;

        private MotorNeturalState(boolean coast) {
            this.coast = coast;
        }


        public NeutralMode getFalconMode() {
            return coast ? NeutralMode.Coast : NeutralMode.Brake;
        }

        public IdleMode getSparkMaxMode() {
            return coast ? IdleMode.kCoast : IdleMode.kBrake;
        }
    }

    public static enum MotorDirection {
        CLOCKWISE(true),
        COUNTERCLOWKWISE(false);

        private final boolean clockwise;

        private MotorDirection(boolean clockwise) {
            this.clockwise = clockwise;
        }


        public TalonFXInvertType getFalconDirection() {
            return clockwise ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise;
        }

        public boolean getSparkMaxDirection() {
            return clockwise;
        }
    }

    public static enum MotorType {
        BRUSHLESS(true),
        BRUSHED(false);

        private final boolean brushless;

        private MotorType(boolean brushless) {
            this.brushless = brushless;
        }


        public CANSparkMaxLowLevel.MotorType getNeoType() {
            return brushless ? CANSparkMaxLowLevel.MotorType.kBrushless : CANSparkMaxLowLevel.MotorType.kBrushed;
        }
    }
}
