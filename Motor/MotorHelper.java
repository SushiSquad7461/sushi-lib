package SushiFrcLib.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;

public class MotorHelper {
    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID) {
        WPI_TalonFX motor = new WPI_TalonFX(canID, "rio");
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, String name) {
        WPI_TalonFX motor = new WPI_TalonFX(canID, name);
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

    public static WPI_TalonFX createFalconMotor(int canID, TalonFXInvertType inversion, String name) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, name);
        motor.setInverted(inversion);
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, inversion);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion, String name) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, inversion, name);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion, NeutralMode neutralMode) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, currentLimit, inversion);
        motor.setNeutralMode(neutralMode);
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion,
            NeutralMode neutralMode, String name) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, currentLimit, inversion, name);
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

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion,
            NeutralMode neutralMode, double p, double i, double d, double f, String name) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, currentLimit, inversion, neutralMode, name);
        motor.config_kP(0, p);
        motor.config_kI(0, i);
        motor.config_kD(0, d);
        motor.config_kF(0, f);
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, Boolean inversion,
            NeutralMode neutralMode, double p, double i, double d, double f, String name, SensorInitializationStrategy init) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, name);
        motor.setInverted(inversion);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        motor.setNeutralMode(neutralMode);
        motor.configIntegratedSensorInitializationStrategy(init);
        motor.config_kP(0, p);
        motor.config_kI(0, i);
        motor.config_kD(0, d);
        motor.config_kF(0, f);
        motor.setSelectedSensorPosition(0);
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, Boolean inversion,
            NeutralMode neutralMode, double p, double i, double d, double f, String name, SensorInitializationStrategy init, double openLoopRampRate) {
        WPI_TalonFX motor = MotorHelper.createFalconMotor(canID, currentLimit, inversion,
            neutralMode, p, i, d, f, name, init);
        motor.configOpenloopRamp(openLoopRampRate);
        return motor;
    }
    

    // Create a new falcon current limit
    public static SupplyCurrentLimitConfiguration createCurrentLimt(int currentLimit) {
        return new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 0);
    }

    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.restoreFactoryDefaults();

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        return motor;
    }

    // Create spark max
    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, int currentLimit, IdleMode idleMode) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.restoreFactoryDefaults();
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setIdleMode(idleMode);

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        return motor;
    }

    // Create spark max
    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, int currentLimit, IdleMode idleMode, double p, double i, double d, double f) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.restoreFactoryDefaults();
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

        return motor;
    }

    // Create spark max
    public static CANSparkMax createSparkMax(int id, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, int openLoopRampRate, int currentLimit) {
        CANSparkMax motor = new CANSparkMax(id, motorType);
        motor.restoreFactoryDefaults();
        motor.setInverted(inverted);
        motor.setOpenLoopRampRate(openLoopRampRate);
        motor.setSmartCurrentLimit(currentLimit);

        if (motorType == CANSparkMaxLowLevel.MotorType.kBrushless) {
            motor.getEncoder().setPosition(0);
        }

        return motor;
    }

    public static PIDController getWpiPidController(double p, double i, double d) {
        return new PIDController(p, i, d);
    }
}
