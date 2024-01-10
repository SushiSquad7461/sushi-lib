package SushiFrcLib.SmartDashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import SushiFrcLib.Control.PIDConfig;


public class PIDTuning {
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private final TunableNumber kF;

    public PIDTuning(String motorName, PIDConfig pid, boolean kTuningMode) {
        this(motorName, pid.P, pid.I, pid.D, pid.F, kTuningMode);
    }

    public PIDTuning(String motorName, double motor_kP, double motor_kI, double motor_kD, double motor_kF, boolean kTuningMode){
        kP = new TunableNumber(motorName + "kP", motor_kP, kTuningMode);
        kI = new TunableNumber(motorName + "kI", motor_kI, kTuningMode);
        kD = new TunableNumber(motorName + "kD", motor_kD, kTuningMode);
        kF = new TunableNumber(motorName + "kF", motor_kF, kTuningMode);
    }

    public void updatePID(CANSparkMax motor){
        if (kP.hasChanged()) motor.getPIDController().setP(kP.get());
        if (kI.hasChanged()) motor.getPIDController().setI(kI.get());
        if (kD.hasChanged()) motor.getPIDController().setD(kD.get());
        if (kF.hasChanged()) motor.getPIDController().setFF(kF.get());
    }

    public void updatePID(TalonFX motor){
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kF.hasChanged()) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kV = kF.get();
            slot0Configs.kP = kP.get();
            slot0Configs.kI = kI.get();
            slot0Configs.kD = kD.get();
            motor.getConfigurator().apply(slot0Configs);
        }
    }
}
