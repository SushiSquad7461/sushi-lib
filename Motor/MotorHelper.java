package SushiFrcLib.Motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;

/**
 * Create Motor Functions deprecated as of Novemeber 2023, use MotorConfig instead
 */
public class MotorHelper {
    public static void setConversionFactor(CANSparkMax motor, double factor) {
        motor.getEncoder().setPositionConversionFactor(factor);
        motor.getEncoder().setVelocityConversionFactor(factor / 60);
    }

    public static void setDegreeConversionFactor(CANSparkMax motor, double gearing) {
        setConversionFactor(motor, 360 / gearing);
    }

    public static CurrentLimitsConfigs createSupplyCurrentLimit(int currentLimit) {
        CurrentLimitsConfigs config = new CurrentLimitsConfigs();

        config.SupplyCurrentLimit = currentLimit;
        config.SupplyCurrentThreshold = currentLimit;
        config.SupplyTimeThreshold = 0;
        config.SupplyCurrentLimitEnable = true;

        return config;
    }

    public static PIDController getWpiPidController(double p, double i, double d) {
        return new PIDController(p, i, d);
    }
}
