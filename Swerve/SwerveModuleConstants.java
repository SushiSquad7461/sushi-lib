package SushiFrcLib.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Wrapper class for swerve module constants.
 */
public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final double angleOffset;
    public final int angleCurrentLimit;
    public final boolean angleInversion;
    public final NeutralMode angleNeutralMode;
    public final double angleP;
    public final double angleI;
    public final double angleD;
    public final double angleF;
    public final int driveCurrentLimit;
    public final boolean driveInversion;
    public final NeutralMode driveNeutralMode;
    public final double driveP;
    public final double driveI;
    public final double driveD;
    public final double driveF;
    public final double openLoopRamp;
    public final double driveGearRatio;
    public final double angleGearRatio;
    public final double wheelCircumference;
    public final boolean cancoderInversion;
    public final String canivoreName;
    public final double maxSpeed;
    public final IdleMode driveIdleMode;
    public final IdleMode angleIdleMode;
    public final double angleRotationsToRadians;
    public final double driveRotationsToMeters;
    public final double angleRPMToRadiansPerSecond;
    public final double driveRPMToMetersPerSecond;


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     */
    public SwerveModuleConstants(
        int driveMotorId, int angleMotorId, int canCoderId, double angleOffset, int angleCurrentLimit, boolean angleInversion, 
        NeutralMode angleNeutralMode, double angleP, double angleI, double angleD, double angleF, String canivoreName, int driveCurrentLimit,
        boolean driveInversion, NeutralMode driveNeutralMode, double driveP, double driveI, double driveD, double driveF, 
        double openLoopRamp, double driveGearRatio, double angleGearRatio, double wheelCircumference, boolean cancoderInversion, 
        double maxSpeed, double angleRotationsToRadians, double driveRotationsToMeters, 
        double angleRPMToRadiansPerSecond, double driveRPMToMetersPerSecond, IdleMode angleIdleMode, IdleMode driveIdleMode
    ) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.cancoderId = canCoderId;
        this.angleOffset = angleOffset;
        this.angleCurrentLimit = angleCurrentLimit;
        this.angleInversion = angleInversion;
        this.angleNeutralMode = angleNeutralMode;
        this.angleP = angleP;
        this.angleI = angleI;
        this.angleD = angleD;
        this.angleF = angleF;
        this.canivoreName = canivoreName;
        this.driveCurrentLimit = driveCurrentLimit;
        this.driveInversion = driveInversion;
        this.driveNeutralMode = driveNeutralMode;
        this.driveP = driveP;
        this.driveI = driveI;
        this.driveD = driveD;
        this.driveF = driveF;
        this.openLoopRamp = openLoopRamp;
        this.driveGearRatio = driveGearRatio;
        this.angleGearRatio = angleGearRatio;
        this.wheelCircumference = wheelCircumference;
        this.cancoderInversion = cancoderInversion;
        this.maxSpeed = maxSpeed;
        this.angleIdleMode = angleIdleMode;
        this.driveIdleMode = driveIdleMode;
        this.angleRPMToRadiansPerSecond = angleRPMToRadiansPerSecond;
        this.driveRPMToMetersPerSecond = driveRPMToMetersPerSecond;
        this.angleRotationsToRadians = angleRotationsToRadians;
        this.driveRotationsToMeters = driveRotationsToMeters;
    }
}
