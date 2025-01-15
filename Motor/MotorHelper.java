package SushiFrcLib.Motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

/**
 * Create Motor Functions deprecated as of Novemeber 2023, use MotorConfig instead
 */
public class MotorHelper {

    /**
     * Sets Spark Max to multiply default position unit of rotations by the conversion factor. 
     * Also sets Spark Max to multiply default velocity unit of RPM by factor and divide by 60 to get target units per second.
     * @param motor
     * @param factor
     */
    public static void setConversionFactor(SparkMax motor, double factor) {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.apply(new EncoderConfig().positionConversionFactor(factor).velocityConversionFactor(factor / 60));
    }

    public static void setDegreeConversionFactor(SparkMax motor, double gearing) {
        setConversionFactor(motor, 360. / gearing);
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
