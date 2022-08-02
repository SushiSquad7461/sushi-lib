package SushiFrcLib.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class MotorConfigFalcon {
    public final int id;
    public final int currentLimit;
    public final NeutralMode neutralMode;
    public final TalonFXInvertType inversion;
    public final double p;
    public final double i;
    public final double d;
    public final double f;

    MotorConfigFalcon(int id, int currentLimit, NeutralMode neutralMode, TalonFXInvertType inversion) {
        this.id = id;
        this.currentLimit = currentLimit;
        this.neutralMode = neutralMode;
        this.inversion = inversion;
        p = 0;
        i = 0;
        d = 0;
        f = 0;
    }


    MotorConfigFalcon(int id, int currentLimit, NeutralMode neutralMode, TalonFXInvertType inversion, double p, double i, double d, double f) {
        this.id = id;
        this.currentLimit = currentLimit;
        this.neutralMode = neutralMode;
        this.inversion = inversion;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public WPI_TalonFX getMotor() {
        return MotorHelper.createFalconMotor(id, currentLimit, neutralMode, inversion, p, i, d, f);
    }
}
