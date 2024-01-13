package SushiFrcLib.Swerve.SwerveModules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeo extends SwerveModule {
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkPIDController drivePID;

    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePID;


    public SwerveModuleNeo(SwerveModuleConstants constants) {
        super(constants);

        driveMotor = constants.getDriveNeo();
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        anglePID.setReference(angleRadians.getRadians(), CANSparkMax.ControlType.kPosition);
        drivePID.setReference(velocityMPS, CANSparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber("Current Encoder Angle: " + swerveModuleConstants.moduleNumber, Rotation2d.fromRadians(angleEncoder.getPosition()).getDegrees());
    }

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getDegrees()); 
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    @Override
    protected double getPositionMeter() {
        return driveEncoder.getPosition();
    }

    @Override
    protected double getVelocityMeter() {
        return driveEncoder.getVelocity();
    }
}
