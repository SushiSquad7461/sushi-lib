package SushiFrcLib.Swerve.SwerveConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import SushiFrcLib.Motor.MotorConfig;
import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Wrapper class for swerve module constants.
 */
public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final int moduleNumber;

    public final Rotation2d angleOffset;

    public final static NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public final static IdleMode angleIdleMode = IdleMode.kCoast;
    public static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    public static IdleMode driveIdleMode = IdleMode.kBrake;

    public final static double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

    public final double driveRotationsToMeters;

    public final boolean swerveTuningMode;

    public final MotorConfig driveConfig;
    public final MotorConfig angleConfig;

    public final SDSModules moduleInfo;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     */
    public SwerveModuleConstants(int moduleNumber, Rotation2d angleOffset, SDSModules moduleInfo,
            boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig) {
                this(moduleNumber, angleOffset, moduleInfo, swerveTuningMode, driveConfig, angleConfig, (moduleNumber * 3) + 1, (moduleNumber * 3) + 2, (moduleNumber * 3) + 3);
    }

    public SwerveModuleConstants(int moduleNumber, Rotation2d angleOffset, SDSModules moduleInfo,
    boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig, int driveMotorId, int angleMotorId, int cancoderId) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        this.moduleInfo = moduleInfo;

        driveRotationsToMeters = wheelCircumference * moduleInfo.driveGearRatio;

        this.swerveTuningMode = swerveTuningMode;

        this.driveConfig = driveConfig;
        this.angleConfig = angleConfig;
        this.cancoderId = cancoderId;
        this.angleMotorId = angleMotorId;
        this.driveMotorId = driveMotorId;
    }

    public static SwerveModuleConstants[] generateConstants(Rotation2d[] angleOffsets, SDSModules moduleInfo,
            boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig anglConfig) {
        SwerveModuleConstants[] ret = new SwerveModuleConstants[4];

        for (int i = 0; i < 4; ++i) {
            ret[i] = new SwerveModuleConstants(
                    i,
                    angleOffsets[i],
                    moduleInfo,
                    swerveTuningMode,
                    driveConfig,
                    anglConfig);
        }

        return ret;
    }

    public SparkMax getDriveNeo() {
        SparkMax neo = new SparkMax(driveMotorId, MotorType.kBrushless);
        driveConfig.setCanSparkMaxConfig(neo, MotorType.kBrushless);
        MotorHelper.setConversionFactor(neo, driveRotationsToMeters);
        return neo;
    }

    public TalonFX getDriveFalcon() {
        TalonFX drive = new TalonFX(driveMotorId, Constants.Ports.CANIVORE_NAME);
        TalonFXConfiguration config = driveConfig.getTalonConfig();
        config.Feedback.SensorToMechanismRatio = moduleInfo.driveGearRatio;
        drive.getConfigurator().apply(config);
        return drive;
    }

    public SparkMax getAngleNeo() {
        SparkMax neo = new SparkMax(angleMotorId, MotorType.kBrushless);
        angleConfig.setCanSparkMaxConfig(neo, MotorType.kBrushless);
        MotorHelper.setDegreeConversionFactor(neo, moduleInfo.angleGearRatio);
        return neo;
    }

    public TalonFX getAngleFalcon() {
        TalonFX angle = new TalonFX(angleMotorId, Constants.Ports.CANIVORE_NAME);
        TalonFXConfiguration config = angleConfig.getTalonConfig();
        MotorHelper.setDegreeConversionFactor(config, moduleInfo.angleGearRatio);
        angle.getConfigurator().apply(config);
        return angle;
    }

    public CANcoder getCanCoder() {
        CANcoder angleEncoder = new CANcoder(cancoderId);

        CANcoderConfiguration config = new CANcoderConfiguration();
        //setting discontinuity point to 1.0 for unsigned [0,1) range
        config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -angleOffset.getRotations();

        angleEncoder.getConfigurator().apply(config);

        return angleEncoder;
    }
}
