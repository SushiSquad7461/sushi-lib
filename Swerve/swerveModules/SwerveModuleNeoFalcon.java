package SushiFrcLib.Swerve.swerveModules;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.CTREModuleState;
import SushiFrcLib.Swerve.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoFalcon extends SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final SwerveModuleConstants moduleConstants;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkMaxPIDController anglePID;

    private Rotation2d lastAngle;

    public SwerveModuleNeoFalcon(SwerveModuleConstants constants) {
        super(constants);

        this.moduleConstants = constants;

        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        lastAngle = getPose().angle;

        resetToAbsolute();
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = CTREModuleState.optimize(state, lastAngle);


        driveMotor.set(ControlMode.Velocity, Conversion.MPSToFalcon(
            state.speedMetersPerSecond, 
            SwerveModuleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        ));

        double angle = Math.abs(state.speedMetersPerSecond) <= moduleConstants.maxSpeed * 0.01
            ? lastAngle.getDegrees()
            :  state.angle.getDegrees();

        anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
        lastAngle = Rotation2d.fromDegrees(angle);

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Mod Drive Encoder: " + moduleNumber,  Conversion.MPSToFalcon(
                state.speedMetersPerSecond, 
                SwerveModuleConstants.wheelCircumference,
                moduleConstants.driveGearRatio
            ));
            SmartDashboard.putNumber("Target Mod Relative Encoder Angle " + moduleNumber, angle);
        }
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), SwerveModuleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        );
        
        Rotation2d rot = new Rotation2d(angleEncoder.getPosition());

        return new SwerveModulePosition(distance, rot);
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getAngle()); 
        lastAngle = Rotation2d.fromDegrees(getAngle());
    }

    @Override
    public void log() {

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Angle Current" + moduleNumber, angleMotor.getOutputCurrent());
            SmartDashboard.putNumber("Current Mod Reltaive Encoder Angle: " + moduleNumber, MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360));
            SmartDashboard.putNumber("Current Mod Reltaive Encoder Angle Non Mod: " + moduleNumber, angleEncoder.getPosition());
            SmartDashboard.putNumber("Current Mod Drive Encoder " + moduleNumber, driveMotor.getSelectedSensorVelocity());
        }
    }
}