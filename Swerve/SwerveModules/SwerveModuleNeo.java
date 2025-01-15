package SushiFrcLib.Swerve.SwerveModules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeo extends SwerveModule {
    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkClosedLoopController drivePID;

    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;


    public SwerveModuleNeo(SwerveModuleConstants constants) {
        super(constants);

        driveMotor = constants.getDriveNeo();
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getClosedLoopController();

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        resetToAbsolute();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        anglePID.setReference(angleRadians.getDegrees(), SparkMax.ControlType.kPosition);
        if(Math.abs(velocityMPS) < 0.1) {
            driveMotor.stopMotor();
        } else {
            drivePID.setReference(velocityMPS, SparkBase.ControlType.kVelocity);
        }
        SmartDashboard.putNumber("Angle Motor Ref: " + swerveModuleConstants.moduleNumber, angleRadians.getDegrees());
        SmartDashboard.putNumber("Drive Motor Ref: " + swerveModuleConstants.moduleNumber, velocityMPS);
    }

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getDegrees()); 
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    @Override
    protected double getPositionMeter() {
        return driveEncoder.getPosition();
    }

    @Override
    protected double getVelocityMeter() {
        return driveEncoder.getVelocity();
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }

    @Override
    public void publishTelemetry(){
        super.publishTelemetry();
        SmartDashboard.putNumber("Current Encoder Angle: " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition());
        SmartDashboard.putNumber("Drive Velocity: " + swerveModuleConstants.moduleNumber, getVelocityMeter());
    }
}
