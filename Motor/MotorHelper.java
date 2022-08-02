package SushiFrcLib.Motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class MotorHelper {
    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, NeutralMode neutralMode, TalonFXInvertType inversion) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        motor.setInverted(inversion);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        motor.setNeutralMode(neutralMode);
        return motor;
    }

    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, NeutralMode neutralMode, TalonFXInvertType inversion, double p, double i, double d, double f) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        motor.setInverted(inversion);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        motor.setNeutralMode(neutralMode);

        motor.config_kP(0, p);
        motor.config_kI(0, i);
        motor.config_kD(0, d);
        motor.config_kF(0, f);

        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, int currentLimit, TalonFXInvertType inversion) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        motor.setInverted(inversion);
        motor.configSupplyCurrentLimit(createCurrentLimt(currentLimit));
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        return motor;
    }

    // Create a new falcon motor
    public static WPI_TalonFX createFalconMotor(int canID, TalonFXInvertType inversion) {
        WPI_TalonFX motor = new WPI_TalonFX(canID);
        motor.configFactoryDefault();
        motor.setSelectedSensorPosition(0);
        motor.setInverted(inversion);
        return motor;
    }

    // Create a new falcon current limit
    public static SupplyCurrentLimitConfiguration createCurrentLimt(int currentLimit) {
        return new SupplyCurrentLimitConfiguration(true, currentLimit-5, currentLimit, 0.75);
    }
}
