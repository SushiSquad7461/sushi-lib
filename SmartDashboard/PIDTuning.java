package SushiFrcLib.SmartDashboard;

import com.revrobotics.CANSparkMax;


public class PIDTuning {
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;

    public PIDTuning(String motorName, double motor_kP, double motor_kI, double motor_kD, boolean kTuningMode){
        kP = new TunableNumber(motorName + "kP", motor_kP, kTuningMode);
        kI = new TunableNumber(motorName + "kI", motor_kI, kTuningMode);
        kD = new TunableNumber(motorName + "kD", motor_kD, kTuningMode);
    }

    public void updatePID(CANSparkMax motor){
        if (kP.hasChanged()) motor.getPIDController().setP(kP.get());
        if (kI.hasChanged()) motor.getPIDController().setP(kI.get());
        if (kD.hasChanged()) motor.getPIDController().setP(kD.get());
    }
}
