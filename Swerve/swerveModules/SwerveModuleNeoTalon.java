package SushiFrcLib.Swerve.SwerveModules;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoTalon extends SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePID;

    private VelocityVoltage drivePID;

    public SwerveModuleNeoTalon(SwerveModuleConstants constants) {
        super(constants);

        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        resetToAbsolute();

        drivePID = new VelocityVoltage(0,0,false,0,0,false,false,false); 
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getRotations()); 
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Angle Current " + swerveModuleConstants.moduleNumber, angleMotor.getOutputCurrent());
            SmartDashboard.putNumber("Current Reltaive Encoder Angle " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360));
            SmartDashboard.putNumber("Current Reltaive Encoder Angle Non Mod " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition());
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, driveMotor.getVelocity().getValue());
        }
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        driveMotor.setControl(
            drivePID.withVelocity(
                Conversion.MPSToFalcon(
                    velocityMPS, 
                    SwerveModuleConstants.wheelCircumference,
                    swerveModuleConstants.moduleInfo.driveGearRatio
                )
            )
        );

        anglePID.setReference(angleRadians.getRadians(), CANSparkMax.ControlType.kPosition);


        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber,  Conversion.MPSToFalcon(
                velocityMPS, 
                SwerveModuleConstants.wheelCircumference,
                swerveModuleConstants.moduleInfo.driveGearRatio
            ));
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angleRadians.getRadians());
        }
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    @Override
    protected double getPositionMeter() {
        return Conversion.rotationsToM(
            driveMotor.getPosition().getValue(), 
            SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.moduleInfo.driveGearRatio
        );
    }

    @Override
    protected double getVelocityMeter() {
        return Conversion.RPSToMPS(
            driveMotor.getVelocity().getValue(), 
            SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.moduleInfo.driveGearRatio
        );
    }
}