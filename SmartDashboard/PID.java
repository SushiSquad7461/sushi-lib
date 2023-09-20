package SushiFrcLib.SmartDashboard;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;


public class PID {
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;

    public PID(double motor_kP, double motor_kI, double motor_kD){
        kP = new TunableNumber("kP", motor_kP, Constants.kTuningMode);
        kI = new TunableNumber("kI", motor_kI, Constants.kTuningMode);
        kD = new TunableNumber("kD", motor_kD, Constants.kTuningMode);
    }

    public void updatePID(CANSparkMax motor){
        if (kP.hasChanged()) motor.getPIDController().setP(kP.get());
        if (kI.hasChanged()) motor.getPIDController().setP(kI.get());
        if (kD.hasChanged()) motor.getPIDController().setP(kD.get());
    }
}
