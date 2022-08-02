package SushiFrcLib.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;

public class MotorHelper {
    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, TalonFXInvertType inversion) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID);
        motor.setInverted(inversion);
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, inversion);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion, NeutralMode neutralMode) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, currentLimit, inversion);
        motor.setNeutralMode(neutralMode);
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion, NeutralMode neutralMode, double p, double i, double d, double f) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, currentLimit, inversion, neutralMode);
        motor.config_kP(0, p);
        motor.config_kI(0, i);
        motor.config_kD(0, d);
        motor.config_kF(0, f);
        return motor;
    }
    

    // Create a new falcon current limit
    public static SupplyCurrentLimitConfiguration createCurrentLimt(int currentLimit) {
        return new SupplyCurrentLimitConfiguration(true, currentLimit-5, currentLimit, 0.75);
    }

    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.restoreFactoryDefaults();

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        motor.burnFlash();
        return motor;
    }

    // Create spark max
    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, int currentLimit, IdleMode idleMode) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setIdleMode(idleMode);

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        motor.burnFlash();

        return motor;
    }

    // Create spark max
    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, int currentLimit, IdleMode idleMode, double p, double i, double d, double f) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setIdleMode(idleMode);

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        motor.getPIDController().setP(p);
        motor.getPIDController().setI(i);
        motor.getPIDController().setD(d);
        motor.getPIDController().setFF(f);

        motor.burnFlash();

        return motor;
    }

    // Create spark max
    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, int openLoopRampRate, int currentLimit) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.setInverted(inverted);
        motor.setOpenLoopRampRate(openLoopRampRate);
        motor.setSmartCurrentLimit(currentLimit);

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        motor.burnFlash();
        return motor;
    }

    public static PIDController getWpiPidController(double p, double i, double d) {
        return new PIDController(p, i, d);
    }
}
