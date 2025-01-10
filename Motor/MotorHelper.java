package SushiFrcLib.Motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;

/**
 * Create Motor Functions deprecated as of Novemeber 2023, use MotorConfig instead
 */
public class MotorHelper {
    public static void setConversionFactor(SparkMax motor, double factor) {
        MotorConfig motorConfig = new MotorConfig(motor.getDeviceId());

        motorConfig.getTalonConfig().Feedback.SensorToMechanismRatio = factor;
        //motor.getEncoder().setVelocityConversionFactor(factor / 60);
    }

    public static void setDegreeConversionFactor(SparkMax motor, double gearing) {
        setConversionFactor(motor, 360 / gearing);
    }

    public static TalonFXConfiguration setConversionFactor(TalonFXConfiguration config, double factor) {
        config.Feedback.SensorToMechanismRatio = 1.0 / factor;
        return config;
    }

    public static TalonFXConfiguration setDegreeConversionFactor(TalonFXConfiguration config, double gearing) {
        return setConversionFactor(config, (360.0 / gearing));
    }

    public static CurrentLimitsConfigs createSupplyCurrentLimit(int currentLimit) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();

        config.SupplyCurrentLimit = currentLimit;
        config.SupplyCurrentLowerLimit = currentLimit;
        config.SupplyCurrentLowerTime = 0;
        config.SupplyCurrentLimitEnable = true;

        return config;
    }

    public static void updateSupplyCurrentLimit(int currentLimit, TalonFXConfiguration config) {
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = 0;

        config.CurrentLimits.SupplyCurrentLowerLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    public static PIDController getWpiPidController(double p, double i, double d) {
        return new PIDController(p, i, d);
    }
}
