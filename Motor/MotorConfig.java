package SushiFrcLib.Motor;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.State.State.MotorDirection;
import SushiFrcLib.State.State.MotorNeturalState;
import SushiFrcLib.State.State.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class MotorConfig {
    public final int id;
    public final int currentLimit;
    public final MotorNeturalState neutralMode;
    public final MotorDirection inversion;
    public final double p;
    public final double i;
    public final double d;
    public final double f;
    public final MotorType type;

    MotorConfig(int id, int currentLimit, MotorNeturalState neutralMode, MotorDirection inversion) {
        this(id, currentLimit, neutralMode, inversion, 0, 0, 0, 0, MotorType.BRUSHLESS);
    }

    MotorConfig(int id, int currentLimit, MotorNeturalState neutralMode, MotorDirection inversion, MotorType type) {
        this(id, currentLimit, neutralMode, inversion, 0, 0, 0, 0, type);
    }

    MotorConfig(int id, int currentLimit, MotorNeturalState neutralMode, MotorDirection inversion, double p, double i, double d, double f) {
        this(id, currentLimit, neutralMode, inversion, p, i, d, f, MotorType.BRUSHLESS);
    }


    MotorConfig(int id, int currentLimit, MotorNeturalState neutralMode, MotorDirection inversion, double p, double i, double d, double f, MotorType type) {
        this.id = id;
        this.currentLimit = currentLimit;
        this.neutralMode = neutralMode;
        this.inversion = inversion;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.type = type;
    }

    public WPI_TalonFX getFalconMotor() {
        return MotorHelper.createFalconMotor(id, currentLimit, inversion.getFalconDirection(), neutralMode.getFalconMode(), p, i, d, f);
    }

    public CANSparkMax getSparkMax() {
        return MotorHelper.createSparkMax(id, type.getNeoType(), inversion.getSparkMaxDirection(), currentLimit, neutralMode.getSparkMaxMode(), p, i, d, f);
    }

    public PIDController getWpiPid() {
        return MotorHelper.getWpiPidController(p, i, d);
    }
}